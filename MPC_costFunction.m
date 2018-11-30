function [Q,R1,R2,P] = MPC_costFunction(abs_flag)
%COSTFUNCTION Defines the cost function of the MPC
% 
% The cost function depends on the state of the ABS. If the ABS is engaged,
% we penalize the the slip error, if not, we penalize the torque error.
sx_norm  = 0.1;
tau_norm = 1000;
Fc_norm  = 1000;

if abs_flag == true % ABS engaged
    q1 = 1;
    q2 = 1;
else
    q1 = 1;
    q2 = 1;
end

Q  = q1 * 1/sx_norm^2;
R1 = q2 * 1/tau_norm^2;
R2 = diag([1/tau_norm^2 1/Fc_norm^2 0]);
P  = diag([0 0 0 0 0]);
end

