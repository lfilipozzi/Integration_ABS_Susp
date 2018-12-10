classdef RMPC < matlab.System & ...
        matlab.system.mixin.Propagates & ...
        matlab.system.mixin.SampleTime
%MPC Provide the MATLAB system to use MPC from the MPCTools toolbox in 
% Simulink. This requires to use interpreted execution. 
%   
%   The function requires the state of the model as an input and a structure
%   which define the parameters used in the model. The equations of motion
%   of the model have to be written in a separate function 
%   x_dot = MPC_EOM(x,u,param).
    
    % Public, tunable properties
    properties
        % Sampling time
        Delta = 0.01;
        % Number of state
        Nx = 5;
        % Number of input
        Nu = 3;
        % Number of disturbance
        Nw = 1;
        % Number of reference signals to follow
        Nr = 2;
        % Prediction horizon
        Nt = 20;
        % Slip target
        sx_ref = -0.3
    end

    properties(Nontunable)
        % Structure array of parameters
        param = struct();
    end

    properties(DiscreteState)
        % System state at the last timestep
        xprev
        % Control input at the last timestep
        uprev
        % Road preview
        roadprev
    end

    % Pre-computed constants
    properties(Access = private)
        QP_TC   % Matrices for QP problem in Torque control mode
        QP_SC   % Matrices for QP problem in Slip control mode
        quadprog_options
    end

    methods(Access = protected)
        %% setupImpl & stepImpl & resetImpl
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            
            % Compute matrices A, B, P, Q, R, K for slip control and torque
            % control mode.
            
            % +---------------------------+
            % |    Linearization point    |
            % +---------------------------+
            % Find steady-state condition for linearization
            % For torque control (ABS disengaged)
            [xss_TC, uss_TC] = steadyState(0,obj.param);
            % For slip control (ABS engaged)
            [xss_SC, uss_SC] = steadyState(-0.25,obj.param);
            
            % +---------------------------+
            % | Linearize and discretize  |
            % +---------------------------+
            [A_TC,B_TC, Bd_TC] = ...
                obj.getLinearizedModel(xss_TC, uss_TC);
            [A_SC,B_SC, Bd_SC] = ...
                obj.getLinearizedModel(xss_SC, uss_SC);
            
            % Extend model
            C = [0 0 0 0 1;
                 0 0 0 0 0;
                 1 0 0 0 0;
                 0 1 0 0 0];
            D = [0 0 0;
                 0 1 0;
                 0 0 0;
                 0 0 0];
            Dd = [0; 0; 0; 0];
            A_TC = [A_TC zeros(obj.Nx, obj.Nr);
                 C          eye(obj.Nr)];
            B_TC = [B_TC; D];
            Bd_TC = [Bd_TC; Dd];
            
            A_SC = [A_SC zeros(obj.Nx, obj.Nr);
                 C          eye(obj.Nr)];
            B_SC = [B_SC; D];
            Bd_SC = [Bd_SC; Dd];
            
            % +---------------------------+
            % |       Cost function       |
            % +---------------------------+
            % Stage cost and terminal cost
            sx_norm   = 0.1;
            tau_norm  = 1000;
            Fc_norm   = 1000;
            puns_norm = 50;
            psp_norm  = 50;
            
            % Weight on states
            qpuns = 50;     % Weight on puns
            qpsp  = 50;     % Weight on qpsp
            % Weights for torque control
            q1_TC = 0;      % Tracking of slip
            q2_TC = 1000;   % Tracking of torque
            % Weights for slip control
            q1_SC = 1000;   % Tracking of slip
            q2_SC = 0;      % Tracking of torque
            % Weight on actuator
            qFc  = 1;   % Weight on rate of change of Fc
            qtau = 100;     % Weight on rate of change of tau
            
            % Torque control
            Q_TC  = diag([...
                0,...   % Penalty on rate of change of sprung mass momentum
                0,...   % Penalty on rate of change of unsprung mass momentum
                0,...   % Penalty on rate of change of suspension displacement
                0,...   % Penalty on rate of change of tire displacmeent
                0,...   % Penalty on rate of change of slip ratio
                q1_TC/sx_norm^2,...     % Penalty on slip tracking error
                q2_TC/tau_norm^2,...    % Penalty on torque target error
                qpsp/psp_norm^2,...     % Penalty on sprung mass momentum
                0]);                    % Penalty on unsprung mass momentum
            
            R_TC  = diag([qtau/tau_norm^2 qFc/Fc_norm^2 0]);
            
            P_TC = zeros(obj.Nx + obj.Nr);
            P_TC(1:5,1:5) = dare(A_TC(1:5,1:5),B_TC(1:5,1:2),...
                Q_TC(1:5,1:5),R_TC(1:2,1:2));
            
            % Slip control
            Q_SC  = diag([...
                0,...   % Penalty on rate of change of sprung mass momentum
                0,...   % Penalty on rate of change of unsprung mass momentum
                0,...   % Penalty on rate of change of suspension displacement
                0,...   % Penalty on rate of change of tire displacmeent
                0,...   % Penalty on rate of change of slip ratio
                q1_SC/sx_norm^2,...     % Penalty on slip tracking error
                q2_SC/tau_norm^2,...    % Penalty on torque target error
                0,...                   % Penalty on the sprung mass momentum
                qpuns/puns_norm^2]);    % Penalty on the unsprung mass momentum
            
            R_SC  = diag([qtau/tau_norm^2 qFc/Fc_norm^2 0]);
            
            P_SC = zeros(obj.Nx + obj.Nr);
            P_SC(1:5,1:5) = dare(A_SC(1:5,1:5),B_SC(1:5,1:2),...
                Q_SC(1:5,1:5),R_SC(1:2,1:2));
            
            % +---------------------------+
            % |    Controller of RMPC     |
            % +---------------------------+
            K_TC = zeros(obj.Nu,9);%obj.Nx);
            K_SC = zeros(obj.Nu,9);%obj.Nx);
            
            % +---------------------------+
            % |   Build structure array   |
            % +---------------------------+
            obj.QP_TC = struct('A',A_TC,'B',B_TC,'Bd',Bd_TC,...
                'P',P_TC,'Q',Q_TC,'R',R_TC,'K',K_TC);
            obj.QP_SC = struct('A',A_SC,'B',B_SC,'Bd',Bd_SC,...
                'P',P_SC,'Q',Q_SC,'R',R_SC,'K',K_SC);
            
            % +---------------------------+
            % |     Quadprog options      |
            % +---------------------------+
            obj.quadprog_options = optimoptions('quadprog');
            obj.quadprog_options.StepTolerance = 1e-18; % Default 1e-12
            obj.quadprog_options.MaxIterations = 1000;   % Default 200
            % obj.quadprog_options.Display = 'none';
            obj.quadprog_options.OptimalityTolerance = 1e-15;   % Default 1e-8
        end

        function [u, solveTime] = stepImpl(obj, tau_ref, ...
                road_prev, x, ABS_flag, U)
            % Implement algorithm. 
            
            % Setup chronometer
            tic;
            time0 = toc;
            
            if ABS_flag == 1    % ABS is not engaged (torque control)
                u = obj.solveMPC(obj.QP_TC, road_prev, x, tau_ref, U);
            else                % ABS is engaged (slip control)
                u = obj.solveMPC(obj.QP_SC, road_prev, x, tau_ref, U);
            end
            
            % Record time
            time1 = toc;
            solveTime = time1 - time0;
            
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.xprev = zeros(obj.Nx,1);
            obj.uprev = zeros(obj.Nu,1);
            obj.roadprev = zeros(1, obj.Nt);
        end
        
        %% Linearize and discretize model
        function [A_disc,B_disc, Bd_disc] = getLinearizedModel(obj, xss, ~)
            % Linearize the model around the operating point (xss, uss)
            
            % Load parameters
            ksi = obj.param.ks;
            bsi = obj.param.bs;
            mi  = obj.param.msp;
            kt  = obj.param.kt;
            mus = obj.param.mus;
            rw  = obj.param.rw;
            B   = obj.param.B;
            C   = obj.param.C;
            D   = obj.param.D;
            Jw  = obj.param.Jw;
            g   = obj.param.g;
            
            % Linearization point
            sxi = xss(5);
            qti = xss(4);
            
            % Velocity treated as parameter in slip equation
            U = 10/3.6;
            
            % Write continuous-time state-space
            A_lin = [ -bsi/mi,  bsi/mus,  ksi,                                                                     0,                                                                                                                                                                               0;
                   bsi/mi, -bsi/mus, -ksi,                                                                    kt,                                                                                                                                                                               0;
                    -1/mi,    1/mus,    0,                                                                     0,                                                                                                                                                                               0;
                        0,   -1/mus,    0,                                                                     0,                                                                                                                                                                               0;
                        0,        0,    0, -(D*kt*rw^2*sin(C*atan(B*sxi))*((Jw*(sxi + 1))/(mi*rw^2) + 1))/(Jw*U), -(rw*((D*Jw*sin(C*atan(B*sxi))*(g*(mi + mus) + kt*qti))/(mi*rw) + (B*C*D*rw*cos(C*atan(B*sxi))*((Jw*(sxi + 1))/(mi*rw^2) + 1)*(g*(mi + mus) + kt*qti))/(B^2*sxi^2 + 1)))/(Jw*U)];
            
            B_lin = [  1,         0, 0;
                -1,         0, 0;
                 0,         0, 0;
                 0,         0, 1;
                 0, rw/(Jw*U), 0];
                 
            Bd_lin = [0; 0; 0; 1; 0];
            
            % Discretize the model
            sys_cont = ss(A_lin, [B_lin Bd_lin], [], []);
            sys_disc = c2d(sys_cont, obj.Delta);
            
            A_disc  = sys_disc.A;
            B_disc  = sys_disc.B(:, 1:size(B_lin,2));
            Bd_disc = sys_disc.B(:, size(B_lin,2)+1:end);
            
        end
        
        %% Build min-max MPC problem
        function u0 = RMPCSolver_solve(obj, P, Q, R, A, B, Bd, K, x0, ...
                umin, umax, wmin, wmax, u3_prev)
            % Solve the min-max MPC problem:
            % min_u max_w xN' P xN + sum_k xk' Q xk + uk' R uk
            % subkect to:
            %       x_k+1 = A x_k + B u_k + Bd w_k
            %       u_k = K x_k + v_k
            %       u3_k = u3_prev(k) (preview on the 3rd input disturbance)
            %       x_0 = x0
            %       u_min <= u_k <= u_max
            %       w_min <= w <= w_max
            
            % Need to redefine Nx, Nu, and Nw since the model has been
            % extended
            Nx_MPC = size(A,1);   % Number of states
            Nu_MPC = size(B,2);   % Number of control inputs
            Nw_MPC = size(Bd,2);  % Number of input disturbances
            Nt_MPC = obj.Nt;
            
            % Define AK
            AK = A + B*K;
            
            % Check size:
            if not(prod(size(umax) == [Nu_MPC, 1]))
                error('Incorect size for uub')
            end
            if not(prod(size(umin) == [Nu_MPC, 1]))
                error('Incorect size for ulb')
            end
            if not(prod(size(wmax) == [Nw_MPC, 1]))
                error('Incorect size for wub')
            end
            if not(prod(size(wmin) == [Nw_MPC, 1]))
                error('Incorect size for wlb')
            end

            % Check min <= max
            if sum(umin > umax)
                error('Unfeasible bounds on u')
            end
            if sum(wmin > wmax)
                error('Unfeasible bounds on w')
            end
            
            % Extend matrices umin, umax to obtain uub and ulb where:
            %       ulb <= u <= uub
            % with:
            %       u = [u0 ... uN-1]^T
            uub = zeros(Nu_MPC*Nt_MPC,1);
            for k = 1:Nt_MPC
                uub((k-1)*Nu_MPC+1:k*Nu_MPC,:) = umax;
            end

            ulb = zeros(Nu_MPC*Nt_MPC,1);
            for k = 1:Nt_MPC
                ulb((k-1)*Nu_MPC+1:k*Nu_MPC,:) = umin;
            end
            
            % Solve the problem by using a batch approch:
            % Write the state equation over the prediction horizon:
            %       x = Sxx x0 + Sxv v + Sxw w
            % where:
            %       x = [x0 ... xN]^T
            %       v = [v0 ... vN-1]^T
            %       w = [w0 ... wN-1]^T

            Sxx = zeros(Nx_MPC*(Nt_MPC+1),Nx_MPC);
            Sxx(1:Nx_MPC,:) = eye(Nx_MPC);
            for i = 1:Nt_MPC
                Sxx(i*Nx_MPC+1:(i+1)*Nx_MPC,:) = AK * Sxx((i-1)*Nx_MPC+1:i*Nx_MPC,:);
            end

            Sxv = zeros(Nx_MPC*(Nt_MPC+1),Nu_MPC*Nt_MPC);
            % Sv(Nx*Nt+1:end,Nu*(Nt-1)+1:end) = B;
            for i = flip(2:Nt_MPC+1)
                Sxv(Nx_MPC*(i-1)+1:Nx_MPC*i, Nu_MPC*(i-2)+1:Nu_MPC*(i-1)) = B;
                for j = flip(1:i-2)
                    Sxv((i-1)*Nx_MPC+1:i*Nx_MPC, (j-1)*Nu_MPC+1:j*Nu_MPC) = ...
                        AK * Sxv((i-1)*Nx_MPC+1:i*Nx_MPC, j*Nu_MPC+1:(j+1)*Nu_MPC);
                end
            end

            Sxw = zeros(Nx_MPC*(Nt_MPC+1),Nw_MPC*Nt_MPC);
            % Sv(Nx*Nt+1:end,Nu*(Nt-1)+1:end) = B;
            for i = flip(2:Nt_MPC+1)
                Sxw(Nx_MPC*(i-1)+1:Nx_MPC*i, Nw_MPC*(i-2)+1:Nw_MPC*(i-1)) = Bd;
                for j = flip(1:i-2)
                    Sxw((i-1)*Nx_MPC+1:i*Nx_MPC, (j-1)*Nw_MPC+1:j*Nw_MPC) = ...
                        AK * Sxw((i-1)*Nx_MPC+1:i*Nx_MPC, j*Nw_MPC+1:(j+1)*Nw_MPC);
                end
            end

            % Write the control input equation over the prediction horizon
            %       u = Sux x0 + Suv v + Suw w
            % where:
            %       u = [u0 ... uN-1]^T
            %       v = [v0 ... vN-1]^T
            %       w = [w0 ... wN-1]^T

            KK = zeros(Nt_MPC*Nu_MPC,Nt_MPC*Nx_MPC);
            for k = 1:Nt_MPC
                KK((k-1)*Nu_MPC+1:k*Nu_MPC,(k-1)*Nx_MPC+1:k*Nx_MPC) = K;
            end

            Sux = KK * Sxx(1:Nt_MPC*Nx_MPC,:);

            Suv = eye(Nt_MPC*Nu_MPC);
            Suv(Nu_MPC+1:end,1:Nu_MPC) = Sux(1:(Nt_MPC-1)*Nu_MPC,:) * B;
            for j = 1:Nt_MPC-1
                Suv(j*Nu_MPC+1:end,(j-1)*Nu_MPC+1:j*Nu_MPC) = Suv(Nu_MPC+1:(Nt_MPC-j+1)*Nu_MPC,1:Nu_MPC);
            end
            
            Suw = zeros(Nt_MPC*Nu_MPC,Nt_MPC*Nw_MPC);
            Suw(Nu_MPC+1:end,1:Nw_MPC) = Sux(1:(Nt_MPC-1)*Nu_MPC,:) * Bd;
            for j = 1:Nt_MPC-1
                Suw(j*Nu_MPC+1:end,(j-1)*Nw_MPC+1:j*Nw_MPC) = Suw(Nu_MPC+1:(Nt_MPC-j+1)*Nu_MPC,1:Nw_MPC);
            end

            % Write cost function over the prediction horizon:
            %       J = x' QQ x + u' RR u
            QQ = zeros(Nx_MPC*(Nt_MPC+1));
            for k = 1:Nt_MPC
                QQ((k-1)*Nx_MPC+1:k*Nx_MPC,(k-1)*Nx_MPC+1:k*Nx_MPC) = Q;
            end
            QQ(Nt_MPC*Nx_MPC+1:(Nt_MPC+1)*Nx_MPC,Nt_MPC*Nx_MPC+1:(Nt_MPC+1)*Nx_MPC) = P;

            RR = zeros(Nu_MPC*Nt_MPC);
            for k = 1:Nt_MPC
                RR((k-1)*Nu_MPC+1:k*Nu_MPC,(k-1)*Nu_MPC+1:k*Nu_MPC) = R;
            end

            % Reformulate the cost function using matrices Hx, Hv, and Hw:
            %       J = || Hx x + Hv v + Hw w ||_2^2
            % From the matrices that have been computed, the cost can be written as:
            %   J = [x0' v' w'] * HH * [x0; v; w]
            % with HH a symmetric positive semi-definit matrix
            % 
            % HH = [(Sxx'*QQ*Sxx + Sux'*RR*Sux) (Sxx'*QQ*Sxv + Sux'*RR*Suv) (Sxx'*QQ*Sxw + Sux'*RR*Suw);
            %       (Sxv'*QQ*Sxx + Suv'*RR*Sux) (Sxv'*QQ*Sxv + Suv'*RR*Suv) (Sxv'*QQ*Sxw + Suv'*RR*Suw);
            %       (Sxw'*QQ*Sxx + Suw'*RR*Sux) (Sxw'*QQ*Sxv + Suw'*RR*Suv) (Sxw'*QQ*Sxw + Suw'*RR*Suw)];
            % 
            % Where:
            %   HH = [ Hx'*Hx   Hx'*Hv   Hx'*Hw;
            %          Hv'*Hx   Hv'*Hv   Hv'*Hw;
            %          Hw'*Hx   Hw'*Hv   Hw'*Hw];

            % HxtHx = Sxx'*QQ*Sxx + Sux'*RR*Sux;
            % HxtHv = Sxx'*QQ*Sxv + Sux'*RR*Suv;
            % HxtHw = Sxx'*QQ*Sxw + Sux'*RR*Suw;
            HvtHv = Sxv'*QQ*Sxv + Suv'*RR*Suv;
            % HvtHw = Sxv'*QQ*Sxw + Suv'*RR*Suw;
            HwtHw = Sxw'*QQ*Sxw + Suw'*RR*Suw;
            HvtHx = Sxv'*QQ*Sxx + Suv'*RR*Sux;
            HwtHx = Sxw'*QQ*Sxx + Suw'*RR*Sux;
            HwtHv = Sxw'*QQ*Sxv + Suw'*RR*Suv;
            
            % Compute all vertices of the disturbance set
            % Create all possible combination of disturbances given the bounds on w
            % over the prediction horizon. Possible combinations are:
            %       w_1 = [wmin ... wmin wmin wmin]^T
            %       w_2 = [wmin ... wmin wmin wmax]^T
            %       w_3 = [wmin ... wmin wmax wmin]^T
            %       w_4 = [wmin ... wmin wmax wmax]^T
            %       ...
            %       w_{2^(Nw*Nt)} = [wmax ... wmax]^T
            % The number of combination is exponential with the prediction horizon and
            % number of disturbances.
            % 
            % To do that we create w_min_max the vector of extreme values for w for one
            % timestep. Then, w_min_max is a cell structure of extreme values for each
            % timestep over the prediction horizon. Finally, w_vertices is the vector
            % of all possible combinations.

            w_min_max = [wmin wmax];
            w_min_max_horizon = cell(Nt_MPC,1);
            for k = 1:Nt_MPC
                w_min_max_horizon{k} = w_min_max;
            end
            w_vertices = allcomb(w_min_max_horizon{:})';
            
            % Each row of w_vertices correspond to a possible combination of
            % disturbance at one timestep of the prediction horizon.

            % TODO: this only work for one disturbance for now (Nw = 1)
            
            % Transform constraint on x0 and u into constraints on x0 and v
            % With the following bounds on u:
            %       ulb <= u <= uub
            % we have:
            %       ulb <= Sux x0 + Suv v + Suw w <= uub
            % or equivalently
            %      -Sux x0 - Suv v <= - ulb + Suw w
            %       Sux x0 + Suv v <=   uub - Suw w

            % Constraints can now be written as:
            %       F x0 + Gv <= m + M w
            F = [-Sux; Sux];
            G = [-Suv; Suv];
            m = [-ulb; uub];
            M = [Suw; -Suw];

            % Constraints can be written as:
            %       F x0 + Gv <= d
            % with:
            %       d = m + min_{w} M*w
            d = m + min(M*w_vertices,[],2);

            % TODO: for now only bounds on u are considered, need to add bounds on x
            % and polytopic constraints ax x +au u <= b
            
            % Transform min-max optimization problem into mp-QP
            % The min-max MPC problem is equivalent to the following QP problem:
            %   min_{v, gamma} || Hx * x + Hv * v + Hw * 0 ||_2^2 + gamma
            %   subject to:   F*v + G*v <= d
            %                 gamma >= gama_min(w) for all w
            % 
            % with gamma_min(w) = || Hx * x + Hv * v + Hw * w ||_2^2 
            %                   - || Hx * x + Hv * v + Hw * 0 ||_2^2
            %                   = w' Hw'Hw w + 2*w'Hw'*(Hx x + Hv v)
            %
            % To obtain an equivalent problem in which the functional does not depends 
            % on the state vector, we change variable
            %       z = v + (Hv'*Hv)^-1 Hv'Hx x
            % and the min max problem becomes
            %   J = x0' Y x0 + min_{z, gamma} 1/2*z'*H*z + gamma
            %   subject to: Gm z + gm*gamma <= Wm + Sm * x
            %               Gc z <= Wc + Sc x
            % with H = 2*Hv'*Hv

            H = 2*HvtHv;

            % After changing variable, the constraints become:
            %   G z <= d + [G (Hv'Hv)^(-1) Hv'Hx - F] x
            % or equavlaently:
            %   Gc z <= Wc + Sc x
            % and for all w vertex of W:
            % 2 w' Hw'Hv z - gamma <= -w' Hw'Hw w + w' 2*[Hw'Hx - Hw'Hv (Hv'Hv)^(-1) Hv'Hx] x
            % which can be written as 2^(Nw*Nt) inequalities as:
            %   Gm z - gamma <= Wm + Sm x

            Gc = G;
            Wc = d;
            Sc = G/HvtHv*HvtHx - F;

            % Check Wm: size error
            Gm = 2 * w_vertices' * HwtHv;
            gm = -ones(2^(Nw_MPC*Nt_MPC),1);
            Wm = zeros(2^(Nw_MPC*Nt_MPC),1);
            for k = 1:2^(Nw_MPC*Nt_MPC)
                Wm(k) = -w_vertices(:,k)' * HwtHw * w_vertices(:,k);
            end
            Sm = 2 * w_vertices' * (HwtHx - HwtHv / HvtHv * HvtHx);
            
            % Reformulate the problem as a quadratic problem and solve it

            % Make sure H is symmetric due to computation error
            H = (H + H')/2;

            H_QP = [H zeros(Nu_MPC*Nt_MPC,1); zeros(1,Nu_MPC*Nt_MPC) 0];
            f_QP = [zeros(Nu_MPC*Nt_MPC,1); 1];
            A_QP = [Gm gm; Gc zeros(2*Nt_MPC*Nu_MPC,1)];    % TODO: 2*Nt*Nu since only bounds on u are considered
            b_QP = [(Wm + Sm*x0); (Wc + Sc*x0)];
            
            % Add equality constraint for road input preview
            % Create Aselect matrix to select the input that correspond to
            % the road input velocity
            A_select = zeros(Nt_MPC,Nu_MPC*Nt_MPC);
            for k = 1:Nt_MPC
                A_select(k, (k-1)*Nu_MPC+3) = 1;
            end
            Aeq_QP = [A_select*Suv zeros(Nt_MPC,1)];
            beq_QP = u3_prev' - A_select*(Sux - Suv * HvtHv \ HvtHx) * x0;
            
            z_QP = quadprog(H_QP,f_QP,A_QP,b_QP,Aeq_QP,beq_QP,[],[],[],...
                obj.quadprog_options);
            z  = z_QP(1:Nu_MPC*Nt_MPC);     % Get rid of gamma
            v  = z - HvtHv \ HvtHx * x0;    % Compute sequence v
            v0 = v(1:Nu_MPC);   % Select the first entry
            u0 = K*x0 + v0;     % Compute u0
        end
        
        %% Solve MPC problem
        function u = solveMPC(obj, QP, road_prev, x, tau_ref, U)
            % Solve the MPC Problem for a given solver
            
            % Compute the extended state for integral formulation
            xaug = [x-obj.xprev;        % State difference
                x(5)-obj.sx_ref;        % Slip target error
                obj.uprev(2)-tau_ref;   % Torque target error
                x(1);                   % Sprung mass momentum
                x(2)];                  % Unsprung mass momentum
            
            % Save the value of the current state for the next timestep
            obj.xprev = x;
            
            % Set bounds on actuators
            [umin, umax] = getActuatorBounds(obj, x);
            
            % Set bound on disturbance
            wmin = -0.03*U;%-0.01;
            wmax = 0.03*U;%0.01;
            
            % Feed the road preview information to the MPC
            u3_prev = road_prev - obj.roadprev;
            % Save road preview for next timestep
            obj.roadprev = road_prev;
            
            % Solve the MPC problem
            du = obj.RMPCSolver_solve(QP.P, QP.Q, QP.R, ...
                    QP.A, QP.B, QP.Bd, QP.K, xaug, ...
                umin, umax, wmin, wmax, u3_prev);
            
            % Compute the new control input
            u = obj.uprev + du;
            
            % Save the new control input for the next timestep
            obj.uprev = u;
            
        end
        
        %% Define actuator bounds
        function [umin, umax] = getActuatorBounds(obj, state)
            
            umax = inf(obj.Nu, 1);
            umin = -umax;
            
            % Unpack parameters:
            a1 = obj.param.a1;
            a2 = obj.param.a2;
            a3 = obj.param.a3;
            a4 = obj.param.a4;
            a5 = obj.param.a5;
            b1 = obj.param.b1;
            b2 = obj.param.b2;
            b3 = obj.param.b3;
            b4 = obj.param.b4;
            b5 = obj.param.b5;
            msp = obj.param.msp;
            mus = obj.param.mus;
            
            % +-----------------------+
            % |   Suspension bounds   |
            % +-----------------------+
            % Define bounds depending on the suspension relative velocity
            pspr = state(1);
            puns = state(2);
            vrel = -(pspr/msp - puns/mus);
            vm = (a1-a4) / (b4-b1);
            vp = (a1-a3) / (b3-b1);
            if vrel < vm
                umin(1) = max(b1 * vrel + a1, b5 * vrel + a5);
                umax(1) = (b4 * vrel + a4);
            elseif vrel <= vp
                umin(1) = (b1 * vrel + a1);
                umax(1) = (b1 * vrel + a1);
            else
                umin(1) = (b3 * vrel + a3);
                umax(1) = min(b1 * vrel + a1, b2 * vrel + a2);
            end
            
            % +-----------------------+
            % |     Brake  bounds     |
            % +-----------------------+
            % Braking torque must always be negative
            umax(2) = 0;
            
            % +-----------------------+
            % | Integral formulation  |
            % +-----------------------+
            % Translate the bounds to convert the constraint on u into a
            % constraint on du = u(k) - u(k-1)
            umin = umin - obj.uprev;
            umax = umax - obj.uprev;
        end
        
        %% Specify sampling time
        function sts = getSampleTimeImpl(obj)
            % Define sampling time
            sts = createSampleTime(obj,'Type','Discrete Periodic',...
              'SampleTime',obj.Delta,'OffsetTime',0);
        end
        
        %% Simulink functions
        function ds = getDiscreteStateImpl(~)
            % Return structure of properties with DiscreteState attribute
            ds = struct([]);
        end

        function flag = isInputSizeLockedImpl(~,~)
            % Return true if input size is not allowed to change while
            % system is running
            flag = true;
        end
        
        function [flag1, flag2, flag3] = isOutputFixedSizeImpl(~)
            flag1 = true;
            flag2 = true;
            flag3 = true;
        end

        function [out1, out2, out3] = getOutputSizeImpl(obj)
            % Return size for each output port
            out1 = [obj.Nu 1];
            out2 = [1 1];
            out3 = [1 1];
        end
        
        function [out1, out2, out3] = getOutputDataTypeImpl(~)
            out1 = 'double';
            out2 = 'double';
            out3 = 'double';
        end
        
        function [c1, c2, c3] = isOutputComplexImpl(~)
            c1 = false;
            c2 = false;
            c3 = false;
        end
    end
end
