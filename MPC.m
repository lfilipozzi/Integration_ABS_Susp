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
        mpc
        fnonlin
    end

    methods(Access = protected)
        %% setupImpl & stepImpl & resetImpl
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            
            obj.mpc = import_mpctools();
            
            % Define the (non-linear) model
            kwargs = struct('funcname', 'ode');
            obj.fnonlin = obj.mpc.getCasadiFunc(@(x,u) MPC_EOM(x,u,obj.param), ...
                [obj.Nx, obj.Nu], {'x', 'u'}, '**', kwargs);
        end

        function u = stepImpl(obj,tau_ref,sx_ref,road_prev,Q,R1,R2,P,x)
            % Implement algorithm. 
            
            % +---------------------------+
            % |       Define model        |
            % +---------------------------+
            % Find steady-state condition for linearization
            % TODO: instead of using zeros(2,1) use preceding value
            [xss, uss] = steadyState(x,zeros(3,1),obj.param);
            
            % Linearize the model
            linmodel = obj.mpc.getLinearizedModel(obj.fnonlin, ...
                {xss, uss}, {'A','B'}, obj.Delta); 
            linmodel.A
            linmodel.B
            linmodel.A(5,:)
            % Define the linearized model (x_dot = A*x+Bu)
            Flin = obj.mpc.getCasadiFunc(@(x,u) linmodel.A*x + linmodel.B*u, ...
                [obj.Nx, obj.Nu], {'x','u'}, {'dintlin'}); 
            
            % +---------------------------+
            % |   Define cost function    |
            % +---------------------------+
            % Stage cost function
            stagecost = @(x,u) ((x(5)-sx_ref)'*Q*(x(5)-sx_ref) + ...
                (u(2) - tau_ref)'*R1*(u(2) - tau_ref) + u'*R2*u);

            % Terminal cost function
            termcost = @(x) (x'*P*x)/2;

            % Define CasADi cost functions
            l = obj.mpc.getCasadiFunc(stagecost, [obj.Nx, obj.Nu], ...
                {'x','u'}, {'l'});
            Vf = obj.mpc.getCasadiFunc(termcost, obj.Nx, {'x'}, {'Vf'});
            
            % +---------------------------+
            % |       Define bounds       |
            % +---------------------------+
            % TODO: use polytopic constraint with e and ef for SAS
            % TODO: use max and min value for input bounds
            lb = struct();
            lb.u = -inf(obj.Nu, obj.Nt);
            lb.x = -inf(1,obj.Nt+1);

            ub = struct();
            ub.u = inf(obj.Nu, obj.Nt);
            ub.u(2) = 0;    % Braking torque always negative
            ub.x = inf(obj.Nx, obj.Nt+1);
            
            % +---------------------------+
            % |    Define constraints     |
            % +---------------------------+
%             e = obj.mpc.getCasadiFunc(@(x,u) MPC_constraint(x,u,obj.param), ...
%                       [obj.Nx, obj.Nu], {'x', 'u'}, {'e'});
%             ef = e; % Use same constraint for terminal state.
            
            % +---------------------------+
            % |        Build solver       |
            % +---------------------------+
            % Define number of state, input and horizon
            N = struct('x', obj.Nx, 'u', obj.Nu, 't', obj.Nt); 
            % Define cost function and bounds
            kwargs = struct('l', l, 'Vf', Vf, 'lb', lb, 'ub', ub, ...
                'verbosity',0); %'e', e, 'ef', ef,...
            % Build MPC solver
            solver = obj.mpc.nmpc('f', Flin, 'N', N, '**', kwargs);
            
            % +---------------------------+
            % |     Solve MPC problem     |
            % +---------------------------+
            % Set initial state to current state
            solver.fixvar('x', 1, x);
            
            % Feed the road preview information to the MPC
            for k = 1:obj.Nt
                solver.fixvar('u', k, road_prev(:,k),3)
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

        function resetImpl(~)
            % Initialize / reset discrete-state properties
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
        
        function flag = isOutputFixedSizeImpl(~)
            flag = true;
        end

        function out = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [obj.Nu 1];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end
        
        function out1 = getOutputDataTypeImpl(~)
            out1 = 'double';
        end
        
        function c1 = isOutputComplexImpl(~)
            c1 = false;
        end
    end
end
