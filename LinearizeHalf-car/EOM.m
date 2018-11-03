%% Define symbolic variables
% Parameters
run ../parameters.m

% State
syms pm pJ pmF pmR qsF qsR qtF qtR theta h U wF wR x

% Inputs
syms viF viR Fcf Fcr tauf taur

%% Dynamic model
% Compute vertical velocities at the front and rear of the sprung mass
vR  = pm/m - b*cos(theta) * pJ/J;
vF  = pm/m + a*cos(theta) * pJ/J;

% Compute distance
h_susp_F = h_susp - qsF;            % Height of the front suspension
h_susp_R = h_susp - qsR;            % Height of the rear suspension
% rwF_unladen = rw + (mF+mus)*g/kt;   % Unladen front wheel radius
% rwR_unladen = rw + (mR+mus)*g/kt;   % Unladen rear wheel radius
rwF = rw - qtF;   % Front tire radius
rwR = rw - qtR;   % Rear tire radius

% Compute suspension and tire forces
% Suspension vertical forces (gauge)
fsF = ksF*qsF + Fcf + bsF*(pmF/mus - vF);
fsR = ksR*qsR + Fcr + bsR*(pmR/mus - vR);

% Tire vertical forces (gauge)
ftF = kt*qtF + bt*(viF - pmF/mus);
ftR = kt*qtR + bt*(viR - pmR/mus);

% Make sure the tire vertical force cannot be negative
% ftF = max(-(mus + mF)*g,ftF);
% ftR = max(-(mus + mR)*g,ftR);

% Compute longitudinal tire force
% Compute longitudinal slip ratio
% Front slip ratio
% den = max(rwF * wF, U);
den = U;
if den ~= 0
    sFx = (rwF * wF - U) / den;
else
    sFx = 0;
end
% Rear slip ratio
% den = max(rwR * wR, U);
den = U;
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

% Compute the state derivatives
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

% Return the state derivative 
dx = [pm_dot pJ_dot pmF_dot pmR_dot qsF_dot qsR_dot qtF_dot qtR_dot ...
    theta_dot h_dot U_dot wF_dot wR_dot x_dot];

% Define states
x = [pm pJ pmF pmR qsF qsR qtF qtR theta h U wF wR x];