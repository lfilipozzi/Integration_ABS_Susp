classdef extendedKalmanFilter < matlab.System & ...
        matlab.system.mixin.Propagates & ...
        matlab.system.mixin.SampleTime & ...
        matlab.system.mixin.CustomIcon 
    % extendedKalmanFilter Simulink implementation of an EKF
    %
    % This files includes a template to implement an extended Kalman filter
    % in Simulink.

    % Public, tunable properties
    properties
        % Number of state of the model
        nb_state  = 5;
        % Number of input of the model
        nb_input  = 1;
        % Number of output of the model
        nb_output = 3;
        % Initial state
        x_init = [0;
                (50/3.6)/0.3250*9.9649;
                (50/3.6)/0.3250;
                (50/3.6)/0.3250;
                (50/3.6)];
        % Sampling time
        Ts = 0.1;
    end
    
    properties(Nontunable)
        % Strurture array of parameters
        param = struct();
    end

    properties(DiscreteState)
        x_apriori
        x_aposteriori
        P_apriori
        P_aposteriori
        u_k
        u_prev
        Q_k
    end

    % Pre-computed constants
    properties(Access = private)
        
    end

    methods(Access = protected)
        function setupImpl(~)
            % Perform one-time calculations, such as computing constants
        end

        function [x_aposteriori, P_aposteriori] = stepImpl(obj,u_k,y_k,...
                Q_k,R_k)
            % Implement algorithm. Return the a posteriori estimate and the
            % covariance matrix at the following timestep
            
            % Shift register: Save the input and covariance matrix for the
            % next timestep and obtain the one from the last timestep
            obj.u_prev  = obj.u_k;  % u_{k-1}
            obj.u_k = u_k;      % u_{k}
            Q_prev  = obj.Q_k;  % Q_{k-1}
            obj.Q_k = Q_k;      % Q_{k}
            
            % Model update
            [A,E] = obj.linearizedStateMatrices();
            obj.x_apriori = obj.forwardEuler_stateEquation(obj.u_prev);
            obj.P_apriori = A*obj.P_aposteriori*A' + E*Q_prev*E';
            
            % Measurement update
            [C,F] = obj.linearizedOutputMatrices();
            L = obj.P_apriori*C' / (C*obj.P_apriori*C' + F*R_k*F');
            obj.x_aposteriori = obj.x_apriori + L * (y_k - ...
                obj.outputEquation(obj.x_apriori,u_k));
            obj.P_aposteriori = obj.P_apriori - L*C*obj.P_apriori;
            
            % Return outputs
            x_aposteriori = obj.x_aposteriori;
            P_aposteriori = obj.P_aposteriori;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            obj.x_aposteriori = obj.x_init;
            obj.P_aposteriori = eye(obj.nb_state)*1e-5;
            obj.u_k = zeros(obj.nb_input,1);
            obj.Q_k = eye(obj.nb_input);
        end
        
        function sts = getSampleTimeImpl(obj)
            % Define sampling time
            sts = createSampleTime(obj,'Type','Discrete Periodic',...
              'SampleTime',obj.Ts,'OffsetTime',0);
        end
        
        function ds = getDiscreteStateImpl(obj)
            % Return structure of properties with DiscreteState attribute
            ds = obj.x_aposteriori;
        end

        function flag = isInputSizeLockedImpl(~,~)
            % Return true if input size is not allowed to change while
            % system is running
            flag = true;
        end

        function icon = getIconImpl(~)
            % Define icon for System block
            icon = 'Extended Kalman Filter';
            % icon = {'My','System'}; % Example: multi-line text icon
        end
        
        function [c1, c2] = isOutputFixedSizeImpl(~)
            % Return true if output size is not allowed to change while
            % system is running
            c1 = true;
            c2 = true;
       end

        function [sz_1, sz_2] = getOutputSizeImpl(obj)
            % Return size for each output port
              sz_1 = [obj.nb_state 1]; 
              sz_2 = [obj.nb_state obj.nb_state]; 

            % Example: inherit size from first input port
            % out = propagatedInputSize(obj,1);
        end
        
        function [out1, out2] = getOutputDataTypeImpl(~)
            out1 = 'double';
            out2 = 'double';
        end
      
        function [c1, c2] = isOutputComplexImpl(~)
            c1 = false;
            c2 = false;
        end
        
        function x_next = forwardEuler_stateEquation(obj,u_prev)
            % Implement forward-Euler approximation of the state equation
            
            x = obj.x_aposteriori;
            h = obj.Ts;
            
            x_dot = obj.stateEquation(x,u_prev);
            
            x_next = x + x_dot * h;
        end
        
        function [A,E] = linearizedStateMatrices(obj)
            % Return the A and E matrices.
            % A is the linearized state equation with respect to the state
            % evaluated at the previous a posteriori estimate.
            % E is the linearized state equation with respect to the noise
            % evaluated at the previous a posteriori estimate.
                        
            % Linearize
            [A,~] = KF_linearization(obj.x_aposteriori, obj.u_prev,...
                obj.param);
            
            % Discretize A and give E
            A = eye(obj.nb_state) + A * obj.Ts;
            E = eye(obj.nb_input);
        end
        
        function [C,F] = linearizedOutputMatrices(obj)
            % Return the C, and F matrices.
            % C is the linearized output equation with respect to the state
            % evaluated at the current a priori estimate.
            % F is the linearized output equation with respect to the noise
            % evaluated at the current a priori estimate.
            
            % Linearize
            [~,C] = KF_linearization(obj.x_apriori, obj.u_k, obj.param);
            
            % Compute F
            F = eye(obj.nb_output);
        end
        
        function x_dot = stateEquation(obj,x,u)
            % State equation: returns x_dot as a function of x and u.
            % This method is called by the Runge-Kutta method or Euler 
            % integration method to obtain the state at the next timestep
            
            % Compute state derivatives
            [x_dot, ~] = KF_EOM(x,u,obj.param);
        end
        
        function y = outputEquation(obj,x,u)
            % Output equation: returns y_k as a function of x_k and u_k
            
            % Compute state derivatives
            [~, y] = KF_EOM(x,u,obj.param);
        end
    end
end
