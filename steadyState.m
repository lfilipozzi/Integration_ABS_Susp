function [xss, uss] = steadyState(sx,param)
%STEADYSTATE_EOM Return steady-state points of the system

% The linarization depends only on sx and qt. Other states and inputs are
% therefore set to zero. Moreover, qt has to be set to zero to obtain
% steady-state condition. Therefore, the linearization depends only on the
% value of sx.

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

%% Compute tire force
mu = D*sin(C*atan(B*sx));
fz = (mus + msp)*g;
fx = mu * fz;

%% Compute steady-state condition
pspr_ss = 0;
puns_ss = 0;
qs_ss   = 0;
qt_ss   = 0;
sx_ss   = sx;

Fc_ss  = 0;
tau_ss = rw*fx * (1+Jw/(msp*rw^2)*(1+sx));
vin_ss = 0;
tau_ref_ss = 0;

xss = [pspr_ss; puns_ss; qs_ss; qt_ss; sx_ss];
uss = [Fc_ss; tau_ss; vin_ss; tau_ref_ss];

end

