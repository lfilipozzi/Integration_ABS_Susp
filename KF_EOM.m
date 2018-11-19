function [state_dot,y] = KF_EOM(state,input,param)
%KF_EOM Return state derivatives and output of the continuous-time system

%% Unpack state and input vector
% Vertical dynamics
pm    = state(1);
pJ    = state(2);
pmF   = state(3);
pmR   = state(4);
qsF   = state(5);
qsR   = state(6);
qtF   = state(7);
qtR   = state(8);
theta = state(9);
% h     = state(10);
% Longitudinal dynamics
U     = state(11);
wF    = state(12);
wR    = state(13);
% x     = state(14);

viF = 0;
viR = 0;
Fcf  = input(1);
Fcr  = input(2);
tauf = input(3);
taur = input(4);

%% Unpack parameters
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

%% Compute vertical velocities at the front and rear of the sprung mass
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

% data = [ -1410, -5008;
%  -720, -3436;
%  -390, -2324;
%  -210, -1792;
%  -90, -1008;
%  -20, -228;
%  20, 228;
%  90, 596;
%  200, 784;
%  390, 1100;
%  760, 1796;
%  1160, 2560];
% v_rel = data(:,1)*1e-3; % Relative velocity (m/s)
% F_damper = 2*data(:,2); % Damper force for one axle (N)
% % TODO check sign: 
% % is it interp1(v_rel,-F_damper,-(pmF/mus - vF),'linear','extrap')
% % or interp1(v_rel,F_damper,(pmF/mus - vF),'linear','extrap')
% F_damper_F = interp1(v_rel,-F_damper,-(pmF/mus - vF),'linear','extrap');
% F_damper_R = interp1(v_rel,-F_damper,-(pmR/mus - vR),'linear','extrap');
% fsF = ksF*qsF + Fcf + F_damper_F;
% fsR = ksR*qsR + Fcr + F_damper_R;

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
if den ~= 0
    sFx = (rwF * wF - U) / den;
else
    sFx = 0;
end
% Rear slip ratio
den = max(rwR * wR, U);
if den ~= 0
    sRx = (rwR * wR - U) / den;
else
    sRx = 0;
end

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
h_dot     = pm/m;

% Longitudinal dynamics
U_dot = 1/(m+2*mus) * (fFx + fRx);
wF_dot = 1/Jw * (tauf - rwF*fFx);
wR_dot = 1/Jw * (taur - rwR*fRx);
x_dot = U;

%% Return the state derivatives
state_dot = [pm_dot; pJ_dot; pmF_dot; pmR_dot;...
    qsF_dot; qsR_dot; qtF_dot; qtR_dot; theta_dot; h_dot;...
    U_dot; wF_dot; wR_dot; x_dot];

%% Return outputs
y = [];

end

