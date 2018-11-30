classdef roadPreview < matlab.System & ...
        matlab.system.mixin.Propagates & ...
        matlab.system.mixin.SampleTime
    % ROADPREVIEW Gives a vector of input to provide a road preview to the
    % MPC.

    % Public, tunable properties
    properties
        Nt_MPC = 20;
        Ts_MPC = 0.01;
    end

    properties(DiscreteState)

    end
    
    properties(Nontunable)
        % Structure array of parameters
        param = struct();
    end

    % Pre-computed constants
    properties(Access = private)

    end

    methods(Access = protected)
        function setupImpl(~)
            % Perform one-time calculations, such as computing constants
        end

        function [VinF, VinR] = stepImpl(obj,vehicle_state)
            % Implement algorithm. 
            
            a = obj.param.a;
            b = obj.param.b;
            z0_road = obj.param.z0_road;
            w_road = obj.param.w_road;
            lambda_road = obj.param.lambda_road;
            d_bump = obj.param.d_bump;
            A_road = obj.param.A_road;
            Phase_road = obj.param.Phase_road;
            n_road = obj.param.n_road;
            roadProfile_index = obj.param.roadProfile_index;
            
            U     = vehicle_state(11);
            x     = vehicle_state(14);
            x_dot     = U;
            
            % Front axle
            VinF = zeros(1,obj.Nt_MPC);
            for i = 1:numel(VinF)
                % Compute longitudinal speed at the axle
                x_axle  = x + a + (i-1)*U*obj.Ts_MPC;
                DxDt = x_dot;

                % Compute road input velocity
                DzDx = roadProfile(x_axle,z0_road,lambda_road,w_road,d_bump,...
                    A_road,Phase_road,n_road,roadProfile_index);
                VinF(i) = DxDt * DzDx;
            end
            
            
            VinR = zeros(1,obj.Nt_MPC);
            for i = 1:numel(VinR)
                % Compute longitudinal speed at the axle
                x_axle  = x - b + (i-1)*U*obj.Ts_MPC;
                DxDt = x_dot;

                % Compute road input velocity
                DzDx = roadProfile(x_axle,z0_road,lambda_road,w_road,d_bump,...
                    A_road,Phase_road,n_road,roadProfile_index);
                VinR(i) = DxDt * DzDx;
            end
        end

        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
        
        %% Specify sample time
        function sts = getSampleTimeImpl(obj)
            % Define sampling time
            sts = createSampleTime(obj,'Type','Discrete Periodic',...
              'SampleTime',obj.Ts_MPC,'OffsetTime',0);
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
        
        function [flag1, flag2] = isOutputFixedSizeImpl(~)
            flag1 = true;
            flag2 = true;
        end

        function [out1, out2] = getOutputSizeImpl(obj)
            % Return size for each output port
            out1 = [1 obj.Nt_MPC];
            out2 = [1 obj.Nt_MPC];

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
    end
end
