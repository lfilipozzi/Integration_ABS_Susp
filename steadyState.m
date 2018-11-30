function [xss, uss] = steadyState(state,input,param)
%STEADYSTATE_EOM Return steady-state points of the system

%% Unstack parameters
ks  = param.ks;
mus = param.mus;
rw  = param.rw;
B   = param.B;
C   = param.C;
D   = param.D;
Jw  = param.Jw;
g   = param.g;
msp = param.msp;

%% Unstack state and input
Fc  = input(1);
vin = input(3);
sx  = state(5);

%% Compute tire force
mu = D*sin(C*atan(B*sx));
fz = (mus + msp)*g;
fx = mu * fz;

%% Compute steady-state condition
pspr_ss = vin * msp;
puns_ss = vin * mus;
qs_ss   = Fc/ks;
qt_ss   = 0;
sx_ss   = sx;

Fc_ss  = Fc;
tau_ss = rw*fx * (1+Jw/(msp*rw^2)*(1+sx));
vin_ss = vin;

xss = [pspr_ss; puns_ss; qs_ss; qt_ss; sx_ss];
uss = [Fc_ss; tau_ss; vin_ss];

end

