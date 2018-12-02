function [state_dot] = MPC_EOM(state,input,param)

%% Unstack parameters
ks  = param.ks;
bs  = param.bs;
kt  = param.kt;
mus = param.mus;
rw  = param.rw;
B   = param.B;
C   = param.C;
D   = param.D;
Jw  = param.Jw;
g   = param.g;
msp = param.msp;

V = 15/3.6;

%% Unstack input vector
Fc  = input(1);
tau = input(2);
vin = input(3);

%% Unstack state variables
% Vertical dynamics
pspr = state(1);
puns = state(2);
qs   = state(3);
qt   = state(4);
sx   = state(5);

% Compute distance
% rw_unladen = rw + (msp+mus)*g/kt;   % Unladen wheel radius
% rw = min(rw - qt, rw_unladen);   % Tire radius

%% Compute suspension and tire forces
% Suspension and tire vertical forces (gauge)
fsusp = ks*qs + Fc + bs*(puns/mus - pspr/msp);
ftire = kt*qt;

%% Compute longitudinal tire force
% Compute friction coefficient (simplified Pacejka's tire model)
mu = D * sin(C * atan(B * sx));

% Compute normal force
fz = ftire + (mus + msp)*g;

% Compute longitudinal force
fx = mu * fz;

%% Compute the state derivatives
pspr_dot = fsusp;
puns_dot = ftire - fsusp;
qs_dot = puns/mus - pspr/msp;
qt_dot = vin - puns/mus;
sx_dot = rw / (Jw*V) * (tau - rw*fx * (1 + Jw/(msp*rw^2) * (1 + sx)));

%% Return the state derivatives
state_dot = [pspr_dot; puns_dot; qs_dot; qt_dot; sx_dot];
end