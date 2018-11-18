classdef MPC < matlab.System & ...
        matlab.system.mixin.Propagates & ...
        matlab.system.mixin.SampleTime
%MPC Provide the MATLAB system to use MPC from the MPCTools toolbox in 
% Simulink. This requires to use interpreted execution. 
%   
%   The function requires the state of the model as an input and a structure
%   which define the parameters used in the model. The equations of motion
%   of the model have to be written in a separate function 
%   x_dot = EOM(x,u,param).
    
    % Public, tunable properties
    properties
        % Sampling time
        Delta = 0.01;
        % Number of state
        Nx = 12;
        % Number of input
        Nu = 4;
        % Prediction horizon
        Nt = 20;
    end

    properties(Nontunable)
        % Strurture array of parameters
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
            obj.fnonlin = obj.mpc.getCasadiFunc(@(x,u) EOM(x,u,obj.param), ...
                [obj.Nx, obj.Nu], {'x', 'u'}, '**', kwargs);
        end

        function u = stepImpl(obj,x)
            % Implement algorithm. 
            
            % +---------------------------+
            % |       Define model        |
            % +---------------------------+
            % Find steady-state condition for linearization
            [xss, uss] = steadyState(x,ones(4,1),obj.param);
            
            % Linearize the model
            linmodel = obj.mpc.getLinearizedModel(obj.fnonlin, ...
                {xss, uss}, {'A', 'B'}, obj.Delta); 

            % Define the linearized model (x_dot = A*x+Bu)
            Flin = obj.mpc.getCasadiFunc(@(x, u) linmodel.A*x + linmodel.B*u, ...
                [obj.Nx, obj.Nu], {'x', 'u'}, {'dintlin'}); 
            
            % +---------------------------+
            % |   Define cost function    |
            % +---------------------------+
            % Stage cost function
            Q = eye(obj.Nx);
            R = eye(obj.Nu);
            stagecost = @(x,u) (x'*Q*x + u'*R*u)/2;

            % Terminal cost function
            P = eye(obj.Nx);
            termcost = @(x) (x'*P*x)/2;

            % Define CasADi cost functions
            l = obj.mpc.getCasadiFunc(stagecost, [obj.Nx, obj.Nu], ...
                {'x','u'}, {'l'});
            Vf = obj.mpc.getCasadiFunc(termcost, obj.Nx, {'x'}, {'Vf'});
            
            % +---------------------------+
            % |       Define bounds       |
            % +---------------------------+
            max_x = Inf;
            max_u = 0.1;

            lb = struct();
            lb.u = -ones(obj.Nu, obj.Nt)*max_u;
            lb.x = -ones(1,obj.Nt+1)*Inf;

            ub = struct();
            ub.u = ones(obj.Nu, obj.Nt)*max_u;
            ub.x = ones(obj.Nx, obj.Nt+1)*max_x;
            
            % +---------------------------+
            % |        Build solver       |
            % +---------------------------+
            % Define number of state, input and horizon
            N = struct('x', obj.Nx, 'u', obj.Nu, 't', obj.Nt); 
            % Define cost function and bounds
            kwargs = struct('l', l, 'Vf', Vf, 'lb', lb, 'ub', ub, ...
                'verbosity',0);
            % Build MPC solver
            solver = obj.mpc.nmpc('f', Flin, 'N', N, '**', kwargs);
            
            % +---------------------------+
            % |     Solve MPC problem     |
            % +---------------------------+
            solver.fixvar('x', 1, x);
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

        function out = getOutputSizeImpl(obj)
            % Return size for each output port
            out = [obj.Nu 1];

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end
    end
end
