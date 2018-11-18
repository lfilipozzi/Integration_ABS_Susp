function [state_dot] = steadyState_EOM(XUout,XUin,param)
%STEADYSTATE_EOM Return the state equation in order to find the
%steady-state points.
%   This function is used by fsolve in order to find the steady-state
%   points. 
%   Inputs are:
%    - XUout: vector of inputs and states to be determined by fsolve in
%      order to find the steady-state points
%    - XUin: vector of inputs and states that has to be given to find the
%      steady-state points
%    - param: structure array of parameters needed to compute the equation
%      of motion

%% Unstack parameters
m      = param.m;
J      = param.J;
a      = param.a;
b      = param.b;
ksR    = param.ksR;
ksF    = param.ksF;
bsF    = param.bsF;
bsR    = param.bsR;
kt     = param.kt;
bt     = param.bt;
mus    = param.mus;
rw     = param.rw;
B      = param.B;
C      = param.C;
D      = param.D;
Jw     = param.Jw;
g      = param.g;
mF     = param.mF;
mR     = param.mR;
h_susp = param.h_susp;

%% Unstack input and state vectors
viF  = 0;
viR  = 0;
Fcf  = XUin(2);
Fcr  = XUin(3);
tauf = XUout(1);
taur = XUout(2);

% Vertical dynamics
pm    = XUout(3);
pJ    = XUout(4);
pmF   = XUout(5);
pmR   = XUout(6);
qsF   = XUout(7);
qsR   = XUout(8);
qtF   = XUout(9);
qtR   = XUout(10);
theta = XUin(1);
% Longitudinal dynamics
U     = XUin(4);
wF    = XUout(11);
wR    = XUout(12);

% Compute vertical velocities at the front and rear of the sprung mass
vR  = pm/m - b*cos(theta) * pJ/J;
vF  = pm/m + a*cos(theta) * pJ/J;

% Compute distance
h_susp_F = h_susp - qsF;            % Height of the front suspension
h_susp_R = h_susp - qsR;            % Height of the rear suspension
rwF_unladen = rw + (mF+mus)*g/kt;   % Unladen front wheel radius
rwR_unladen = rw + (mR+mus)*g/kt;   % Unladen rear wheel radius
rwF = min(rw - qtF, rwF_unladen);   % Front tire radius
rwR = min(rw - qtR, rwR_unladen);   % Rear tire radius

%% Compute suspension and tire forces
% Suspension vertical forces (gauge)
fsF = ksF*qsF + Fcf + bsF*(pmF/mus - vF);
fsR = ksR*qsR + Fcr + bsR*(pmR/mus - vR);

% Tire vertical forces (gauge)
ftF = kt*qtF + bt*(viF - pmF/mus);
ftR = kt*qtR + bt*(viR - pmR/mus);

% Make sure the tire vertical force cannot be negative
ftF = max(-(mus + mF)*g,ftF);
ftR = max(-(mus + mR)*g,ftR);

%% Compute longitudinal tire force
% Compute longitudinal slip ratio
% Front slip ratio
den = max(rwF * wF, U);
% if den ~= 0
    sFx = (rwF * wF - U) / den;
% else
%     sFx = 0;
% end
% Rear slip ratio
den = max(rwR * wR, U);
% if den ~= 0
    sRx = (rwR * wR - U) / den;
% else
%     sRx = 0;
% end

% Compute friction coefficient (simplified Pacejka's tire model)
muF = D * sin(C * atan(B * sFx));
muR = D * sin(C * atan(B * sRx));

% Compute normal force
fFz = ftF + (mus + mF)*g;
fRz = ftR + (mus + mR)*g;

% Compute longitudinal force
fFx = fFz * muF;
fRx = fRz * muR;


%% Compute the state derivatives
% Vertical dynamics
pm_dot    = fsR + fsF;
pJ_dot    = a*cos(theta) * fsF - b*cos(theta) * fsR ...
          + (h_susp_F + rwF - a*sin(theta)) * fFx ...
          + (h_susp_R + rwR + b*sin(theta)) * fRx;
pmF_dot   = ftF - fsF;
pmR_dot   = ftR - fsR;
qsF_dot   = pmF/mus - vF;
qsR_dot   = pmR/mus - vR;
qtF_dot   = viF - pmF/mus;
qtR_dot   = viR - pmR/mus;
theta_dot = pJ/J;

% Longitudinal dynamics
U_dot = 1/(m+2*mus) * (fFx + fRx);
wF_dot = 1/Jw * (tauf - rwF*fFx);
wR_dot = 1/Jw * (taur - rwR*fRx);

%% Return the state derivatives
state_dot = [pm_dot;
    pJ_dot;
    pmF_dot; 
    pmR_dot;
    qsF_dot; 
    qsR_dot; 
    qtF_dot; 
    qtR_dot; 
    theta_dot;
    U_dot; 
    wF_dot; 
    wR_dot];
end