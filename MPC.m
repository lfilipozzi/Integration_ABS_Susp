classdef MPC < matlab.System & ...
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
    end

    % Pre-computed constants
    properties(Access = private)
        solver_TC_vNeg  % Solver use for torque control with:       v <  v-
        solver_TC_vNeu  % Solver use for torque control with: v- <= v <= v+
        solver_TC_vPos  % Solver use for torque control with: v+ <  v-
        solver_SC_vNeg  % Solver use for torque control with:       v < v-
        solver_SC_vNeu  % Solver use for torque control with: v- <= v <= v+
        solver_SC_vPos  % Solver use for torque control with: v+ <  v
    end

    methods(Access = protected)
        %% setupImpl & stepImpl & resetImpl
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            
            % Build the different solvers of the MPC. Six solvers are
            % built: 3 for torque control (when the slip is low) to deal
            % with the different constraints and 3 for slip control (ABS
            % engaged):
            
            % Unpack parameters:
%             a1 = obj.param.a1;
%             a2 = obj.param.a2;
%             a3 = obj.param.a3;
%             a4 = obj.param.a4;
%             a5 = obj.param.a5;
%             b1 = obj.param.b1;
%             b2 = obj.param.b2;
%             b3 = obj.param.b3;
%             b4 = obj.param.b4;
%             b5 = obj.param.b5;
%             msp = obj.param.msp;
%             mus = obj.param.mus;
            
            % +---------------------------+
            % |    Linearization point    |
            % +---------------------------+
            % Find steady-state condition for linearization
            % For torque control (ABS disengaged)
            [xss, uss] = steadyState(0,obj.param);
            ss_condition_TC = struct('xss',xss,'uss',uss);
            % For slip control (ABS engaged)
            [xss, uss] = steadyState(-0.35,obj.param);
            ss_condition_SC = struct('xss',xss,'uss',uss);
            
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
            qFc  = 0.01;    % Weight on Fc
            qtau = 1;       % Weight on tau
            
            Q_TC  = diag([...
                qpsp/psp_norm^2,...   % Penalty on rate of change of sprung mass momentum
                0,...   % Penalty on rate of change of unsprung mass momentum
                0,...   % Penalty on rate of change of suspension displacement
                0,...   % Penalty on rate of change of tire displacmeent
                0,...   % Penalty on rate of change of slip ratio
                q1_TC/sx_norm^2, ...    % Penalty on slip tracking error
                q2_TC/tau_norm^2]);     % Penalty on torque target error
            R_TC  = diag([qtau/tau_norm^2 qFc/Fc_norm^2]);
            P_TC  = diag([0 0 0 0 0 0 0]);
            
            Q_SC  = diag([...
                0,...   % Penalty on rate of change of sprung mass momentum
                qpuns/puns_norm^2,...   % Penalty on rate of change of unsprung mass momentum
                0,...   % Penalty on rate of change of suspension displacement
                0,...   % Penalty on rate of change of tire displacmeent
                0,...   % Penalty on rate of change of slip ratio
                q1_SC/sx_norm^2, ...    % Penalty on slip tracking error
                q2_SC/tau_norm^2]);     % Penalty on torque target error
            R_SC  = diag([qtau/tau_norm^2 qFc/Fc_norm^2]);
            P_SC  = diag([0 0 0 0 0 0 0]);
            
            % x(1) is the sprung mass momentum
            % x(2) is the unpsrung mass momentum
            % x(5) is the longitudinal slip sx
            % x(6) is the slip target error
            % x(7) is the torque target error
            % u(2) is the brake torque
            
            % For torque control (ABS disengaged)
            stagecost_TC = @(x,u) (...
                x'*Q_TC*x + ...
                u(1:2)'*R_TC*u(1:2));
            termcost_TC = @(x) (x'*P_TC*x)/2;
            % For slip control (ABS engaged)
            stagecost_SC = @(x,u) (...
                x'*Q_SC*x + ...
                u(1:2)'*R_SC*u(1:2));
            termcost_SC = @(x) (x'*P_SC*x)/2;
            
            % +---------------------------+
            % |     Define constraints    |
            % +---------------------------+
            % Define constraints depending on the suspension relative
            % velocity
            % u(1) is the suspension active force Fc and
            % (x(1)/msp-x(2)/mus) is the suspension relative velocity
%             constraint_vNeg = @(x,u) ...
%                 [-u(1) + (b1 * (x(1)/msp-x(2)/mus) + a1);
%                   u(1) - (b4 * (x(1)/msp-x(2)/mus) + a4);
%                  -u(1) + (b5 * (x(1)/msp-x(2)/mus) + a5)];
%             constraint_vNeu = @(x,u) ...
%                 [ u(1) - (b1 * (x(1)/msp-x(2)/mus) + a1)
%                  -u(1) + (b1 * (x(1)/msp-x(2)/mus) + a1)];
%             constraint_vPos = @(x,u) ...
%                 [ u(1) - (b1 * (x(1)/msp-x(2)/mus) + a1);
%                   u(1) - (b2 * (x(1)/msp-x(2)/mus) + a2);
%                  -u(1) + (b3 * (x(1)/msp-x(2)/mus) + a3);];
            
            % +---------------------------+
            % |       Build solvers       |
            % +---------------------------+
            % Build the solvers
            obj.solver_TC_vNeg = obj.buildMPCSolver(ss_condition_TC, stagecost_TC, termcost_TC);
            obj.solver_TC_vNeu = obj.buildMPCSolver(ss_condition_TC, stagecost_TC, termcost_TC);
            obj.solver_TC_vPos = obj.buildMPCSolver(ss_condition_TC, stagecost_TC, termcost_TC);
            obj.solver_SC_vNeg = obj.buildMPCSolver(ss_condition_SC, stagecost_SC, termcost_SC);
            obj.solver_SC_vNeu = obj.buildMPCSolver(ss_condition_SC, stagecost_SC, termcost_SC);
            obj.solver_SC_vPos = obj.buildMPCSolver(ss_condition_SC, stagecost_SC, termcost_SC);
        end

        function [u, solveTime, solverUsed] = stepImpl(obj, tau_ref, ...
                road_prev, x, ABS_flag)
            % Implement algorithm. 
            
            tic;
            time0 = toc;
            
            % Compute the extended state for integral formulation
            xaug = [x-obj.xprev;
                x(5)-obj.sx_ref;
                obj.uprev(2)-tau_ref];
            
            % Save the value of the current state for the next timestep
            obj.xprev = x;
            
            % Compute suspension relative velocity
            pspr = x(1);
            puns = x(2);
            vrel = pspr/obj.param.msp - puns/obj.param.mus;
            vm = (obj.param.a1-obj.param.a4) / (obj.param.b4-obj.param.b1);
            vp = (obj.param.a1-obj.param.a3) / (obj.param.b3-obj.param.b1);
            
            if ABS_flag == 1    % ABS is not engaged (torque control)
                if vrel < vm
                    du = solveMPC(obj, obj.solver_TC_vNeg, road_prev, xaug);
                    solverUsed = 1;
                elseif vrel <= vp
                    du = solveMPC(obj, obj.solver_TC_vNeu, road_prev, xaug);
                    solverUsed = 2;
                else
                    du = solveMPC(obj, obj.solver_TC_vPos, road_prev, xaug);
                    solverUsed = 3;
                end
            else                % ABS is engaged slip control)
                if vrel < vm
                    du = solveMPC(obj, obj.solver_SC_vNeg, road_prev, xaug);
                    solverUsed = 4;
                elseif vrel <= vp
                    du = solveMPC(obj, obj.solver_SC_vNeu, road_prev, xaug);
                    solverUsed = 5;
                else
                    du = solveMPC(obj, obj.solver_SC_vPos, road_prev, xaug);
                    solverUsed = 6;
                end
            end
            
            % Compute the new control input
            u = obj.uprev + du;
            
            % Save the new contorl input for the next timestep
            obj.uprev = u;
            
            time1 = toc;
            
            solveTime = time1 - time0;
            
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.xprev = zeros(obj.Nx,1);
            obj.uprev = zeros(obj.Nu,1);
        end
        
        %% Build MPC solver
        function solver = buildMPCSolver(obj, ss_condition, ...
                stagecost, termcost)
            % This function is used to build the MPC solver for a given
            % cost function, a given constraint function and a given
            % linearizing point.
            % This MPC has an integral formulation in order to follow a
            % requested target.
            
            % Import MPC Tools
            mpc = import_mpctools();
            
            % +---------------------------+
            % |       Define model        |
            % +---------------------------+
            % Define the (non-linear) model
            kwargs = struct('funcname', 'ode');
            fnonlin = mpc.getCasadiFunc(@(x,u) MPC_EOM(x,u,obj.param), ...
                [obj.Nx, obj.Nu], {'x', 'u'}, '**', kwargs);
            
            % Linearize and discretize the model
            xss = ss_condition.xss;
            uss = ss_condition.uss;
            linmodel = mpc.getLinearizedModel(fnonlin, ...
                {xss, uss}, {'A','B'}, obj.Delta);
            
            % Define the linearized model (x_dot = A*x+Bu) with integral
            % formulation
            C = [0 0 0 0 1;
                 0 0 0 0 0];
            D = [0 0 0;
                 0 1 0];
            A = [linmodel.A zeros(obj.Nx, obj.Nr);
                 C          eye(obj.Nr)];
            B = [linmodel.B; D];
            Flin = mpc.getCasadiFunc(@(x,u) A*x + B*u, ...
                [obj.Nx + obj.Nr, obj.Nu], {'x','u'}, {'dintlin'}); 
            
            % +---------------------------+
            % |   Define cost function    |
            % +---------------------------+
            % Convert cost functions into CasADi functions
            l  = mpc.getCasadiFunc(stagecost, [obj.Nx + obj.Nr, obj.Nu], ...
                {'x','u'}, {'l'});
            Vf = mpc.getCasadiFunc(termcost, obj.Nx + obj.Nr, {'x'}, {'Vf'});
            
            % +---------------------------+
            % |       Define bounds       |
            % +---------------------------+
            lb = struct();
            lb.u = -inf(obj.Nu, obj.Nt);
            lb.x = -inf(obj.Nx + obj.Nr, obj.Nt+1);

            ub = struct();
            ub.u = inf(obj.Nu, obj.Nt);
            ub.u(2) = 0;    % Braking torque is always negative
            ub.x = inf(obj.Nx + obj.Nr, obj.Nt+1);
            
            % +---------------------------+
            % |        Build solver       |
            % +---------------------------+
            % Define number of state, input and horizon
            N = struct('x', obj.Nx + obj.Nr, 'u', obj.Nu, 't', obj.Nt); 
            % Define cost function and bounds
            kwargs = struct('l', l, 'Vf', Vf, 'lb', lb, 'ub', ub, ...
                'verbosity',0);
            % Build MPC solver
            solver = mpc.nmpc('f', Flin, 'N', N, '**', kwargs);
        end
        
        %% Solve MPC problem
        function u = solveMPC(obj, solver, road_prev, x)
            % Solve the MPC Problem for a given solver
            
            % Set initial state to current state
            solver.fixvar('x', 1, x);
            
            % Feed the road preview information to the MPC
            for k = 1:obj.Nt
                solver.fixvar('u', k, road_prev(:,k), 3)
            end
            
            % Solve the MPC problem
            solver.solve();

            % Return error if the problem has not been solved
            if ~isequal(solver.status, 'Solve_Succeeded')
                warning('Solver failed!');
            end

            % Return the first control input
            u = solver.var.u(:,1);
        end
        
        %% Specify sample time
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

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
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
