function [xss, uss] = steadyState(state,input,param)
%STEADYSTATE_EOM Return steady-state points of the system
%   This function provides the steady-state point for the linearization of
%   the system.
%   Inputs are:
%    - state: the state vector of the system
%    - input: the input vector of the system
%    - param: structure array of parameters of the system
%   Outputs are:
%    - xss: steady-state state vector
%    - uss: steady-state input vector

%% Unstack state vector
% Vertical dynamics
% pm    = state(1);
% pJ    = state(2);
% pmF   = state(3);
% pmR   = state(4);
% qsF   = state(5);
% qsR   = state(6);
% qtF   = state(7);
% qtR   = state(8);
theta = state(9);
% h     = state(10);
% Longitudinal dynamics
U     = state(11);
% wF    = state(12);
% wR    = state(13);
% x     = state(14);

%% Unstack input vector
Fcf  = input(1);
Fcr  = input(2);
% tauf = input(3);
% taur = input(4);

%% Write set of non-linear equations to solve
% Select the variables that must be given to solve the set of equation. 
% Here, we have: XUin = [theta Fcf Fcr U]^T
XUin = [theta; Fcf; Fcr; U];

% Define the function to give to fsolve
fun = @(XUout) steadyState_EOM(XUout,XUin,param);

% Define solution guess
rw = param.rw;
x0 = [0;    % tauf
    0;      % taur
    0;      % pm
    0;      % pJ
    0;      % pmF
    0;      % pmR
    0;      % qsF
    0;      % qsR
    0;      % qtF
    0;      % qtR
    U/rw;   % wF
    U/rw];  % wR

% Find the steady-state point
XUout = fsolve(fun,x0);

% Rename states and inputs
tauf = XUout(1);
taur = XUout(2);
pm   = XUout(3);
pJ   = XUout(4);
pmF  = XUout(5);
pmR  = XUout(6);
qsF  = XUout(7);
qsR  = XUout(8);
qtF  = XUout(9);
qtR  = XUout(10);
wF   = XUout(11);
wR   = XUout(12);

% Check that we obtain zero for the state-derivative
% steadyState_EOM(XUout,XUin,param)

%% Return steady-state states and inputs in the correct order
% Stack the steady-state state vector
% x = [pm pJ pmF pmR qsF qsR qtF qtR theta U wF wR]^T
xss = [pm pJ pmF pmR qsF qsR qtF qtR theta U wF wR]';

% Stack the steady-state input vector
% u = [Fcf Fcr tauf taur]
uss = [Fcf Fcr tauf taur]';

%% 
pm = 0;
pJ = 0;
pmF = 0;
pmR = 0;
qsF = 0;
qsR = 0;
qtF = 0;
qtR = 0;
theta = 0;
U = state(11);
wF = U/rw;
wR = U/rw;

warning('simple linearization')
xss = [pm pJ pmF pmR qsF qsR qtF qtR theta U wF wR]';
uss = [0 0 0 0]';

end

