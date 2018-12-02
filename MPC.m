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
        % Prediction horizon
        Nt = 20;
    end

    properties(Nontunable)
        % Structure array of parameters
        param = struct();
    end

    properties(DiscreteState)

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
            
            % +---------------------------+
            % |    Linearization point    |
            % +---------------------------+
            % Find steady-state condition for linearization
            % For torque control (ABS disengaged)
            [xss, uss] = steadyState(0,obj.param);
            ss_condition_TC = struct('xss',xss,'uss',uss);
            % For slip control (ABS engaged)
            [xss, uss] = steadyState(-0.3,obj.param);
            ss_condition_SC = struct('xss',xss,'uss',uss);
            
            % +---------------------------+
            % |       Cost function       |
            % +---------------------------+
            % Stage cost and terminal cost
            sx_norm  = 0.1;
            tau_norm = 1000;
            Fc_norm  = 1000;
            
            sx_ref = -0.3;
            tau_ref = -200;
            
            q1 = 1*1000;
            q2 = 0*1000;
            
            Q  = q1 * 1/sx_norm^2;
            R1 = q2 * 1/tau_norm^2;
            R2 = diag([1/tau_norm^2 0.01/Fc_norm^2 0]);
            P  = diag([0 0 0 0 0]);
            
            % For torque control (ABS disengaged)
            stagecost_TC = @(x,u) ((x(5)-sx_ref)'*Q*(x(5)-sx_ref) + ...
                (u(2) - tau_ref)'*R1*(u(2) - tau_ref) + u'*R2*u);
            termcost_TC = @(x) (x'*P*x)/2;
            % For slip control (ABS engaged)
            stagecost_SC = @(x,u) ((x(5)-sx_ref)'*Q*(x(5)-sx_ref) + ...
                (u(2) - tau_ref)'*R1*(u(2) - tau_ref) + u'*R2*u);
            termcost_SC = @(x) (x'*P*x)/2;
            
            % +---------------------------+
            % |     Define constraints    |
            % +---------------------------+
            % Define constraints depending on the suspension relative
            % velocity
            % u(1) is the suspension active force Fc and
            % (x(1)/msp-x(2)/mus) is the suspension relative velocity
            constraint_vNeg = @(x,u) ...
                [-u(1) + (b1 * (x(1)/msp-x(2)/mus) + a1);
                  u(1) - (b4 * (x(1)/msp-x(2)/mus) + a4);
                 -u(1) + (b5 * (x(1)/msp-x(2)/mus) + a5)];
            constraint_vNeu = @(x,u) ...
                [ u(1) - (b1 * (x(1)/msp-x(2)/mus) + a1)
                 -u(1) + (b1 * (x(1)/msp-x(2)/mus) + a1)];
            constraint_vPos = @(x,u) ...
                [ u(1) - (b1 * (x(1)/msp-x(2)/mus) + a1);
                  u(1) - (b2 * (x(1)/msp-x(2)/mus) + a2);
                 -u(1) + (b3 * (x(1)/msp-x(2)/mus) + a3);];
            
            % +---------------------------+
            % |       Build solvers       |
            % +---------------------------+
            % Build the solvers
            obj.solver_TC_vNeg = obj.buildMPCSolver(ss_condition_TC, stagecost_TC, termcost_TC, constraint_vNeg);
            obj.solver_TC_vNeu = obj.buildMPCSolver(ss_condition_TC, stagecost_TC, termcost_TC, constraint_vNeu);
            obj.solver_TC_vPos = obj.buildMPCSolver(ss_condition_TC, stagecost_TC, termcost_TC, constraint_vPos);
            obj.solver_SC_vNeg = obj.buildMPCSolver(ss_condition_SC, stagecost_SC, termcost_SC, constraint_vNeg);
            obj.solver_SC_vNeu = obj.buildMPCSolver(ss_condition_SC, stagecost_SC, termcost_SC, constraint_vNeu);
            obj.solver_SC_vPos = obj.buildMPCSolver(ss_condition_SC, stagecost_SC, termcost_SC, constraint_vPos);
        end

        function [u, solveTime, solverUsed] = stepImpl(obj, road_prev, ...
                x, ABS_flag)
            % Implement algorithm. 
            
            tic;
            time0 = toc;
            
            % Compute suspension relative velocity
            pspr = x(1);
            puns = x(2);
            vrel = pspr/obj.param.msp - puns/obj.param.mus;
            vm = (obj.param.a1-obj.param.a4) / (obj.param.b4-obj.param.b1);
            vp = (obj.param.a1-obj.param.a3) / (obj.param.b3-obj.param.b1);
            
            if ABS_flag == 1    % ABS is not engaged (torque control)
                if vrel < vm
                    u = solveMPC(obj, obj.solver_TC_vNeg, road_prev, x);
                    solverUsed = 1;
                elseif vrel <= vp
                    u = solveMPC(obj, obj.solver_TC_vNeu, road_prev, x);
                    solverUsed = 2;
                else
                    u = solveMPC(obj, obj.solver_TC_vPos, road_prev, x);
                    solverUsed = 3;
                end
            else                % ABS is engaged slip control)
                if vrel < vm
                    u = solveMPC(obj, obj.solver_TC_vNeg, road_prev, x);
                    solverUsed = 4;
                elseif vrel <= vp
                    u = solveMPC(obj, obj.solver_TC_vNeu, road_prev, x);
                    solverUsed = 5;
                else
                    u = solveMPC(obj, obj.solver_TC_vPos, road_prev, x);
                    solverUsed = 6;
                end
            end
            
            time1 = toc;
            
            solveTime = time1 - time0;
            
        end

        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
        
        %% Build MPC solver
        function solver = buildMPCSolver(obj, ss_condition, ...
                stagecost, termcost, constraint)
            % This function is used to build the MPC solver for a given
            % cost function, a given constraint function and a given
            % linearizing point
            
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
            linmodel = mpc.getLinearizedModel(fnonlin, ...
                {ss_condition.xss, ss_condition.uss}, {'A','B'}, ...
                obj.Delta); 
            
            % Define the linearized model (x_dot = A*x+Bu)
            Flin = mpc.getCasadiFunc(@(x,u) linmodel.A*x + linmodel.B*u, ...
                [obj.Nx, obj.Nu], {'x','u'}, {'dintlin'}); 
            
            % +---------------------------+
            % |   Define cost function    |
            % +---------------------------+
            % Convert cost functions into CasADi functions
            l  = mpc.getCasadiFunc(stagecost, [obj.Nx, obj.Nu], ...
                {'x','u'}, {'l'});
            Vf = mpc.getCasadiFunc(termcost, obj.Nx, {'x'}, {'Vf'});
            
            % +---------------------------+
            % |       Define bounds       |
            % +---------------------------+
            lb = struct();
            lb.u = -inf(obj.Nu, obj.Nt);
            lb.x = -inf(1,obj.Nt+1);

            ub = struct();
            ub.u = inf(obj.Nu, obj.Nt);
            ub.u(2) = 0;    % Braking torque is always negative
            ub.x = inf(obj.Nx, obj.Nt+1);
            
            % +---------------------------+
            % |    Define constraints     |
            % +---------------------------+
            % Convert constraint functions into CasADi function
            e = mpc.getCasadiFunc(constraint, ...
                      [obj.Nx, obj.Nu], {'x', 'u'}, {'e'});
            ef = e; % Use same constraint for terminal state.
            
            % +---------------------------+
            % |        Build solver       |
            % +---------------------------+
            % Define number of state, input and horizon
            N = struct('x', obj.Nx, 'u', obj.Nu, 't', obj.Nt); 
            % Define cost function and bounds
%             kwargs = struct('l', l, 'Vf', Vf, 'lb', lb, 'ub', ub, ...
%                 'e', e, 'ef', ef, 'verbosity',0);
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
