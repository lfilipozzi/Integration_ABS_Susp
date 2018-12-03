clear; clc;
close all;

%% Obtain symbolic Jacobian matrics
EOMQuarterCar

% Define inputs and outputs
u = taui;
y = sxi;

% Compute the symbolic Jacobian matrices
A_sym = simplify(jacobian(state_dot, state));
B_sym = simplify(jacobian(state_dot,u));
C_sym = simplify(jacobian(y,state));
D_sym = simplify(jacobian(y,u));

%% Load parameters
run ../parameters

%% Define operating point
% The slip operating point should be close to the value that provides
% maximum slip but in the linear region
sx_astr = (1/B) * tan(pi/(2*C));
sxi = -0.9*sx_astr;
qti = 0;

U = 30/3.6; % Velocity used as parameter in the slip EOM

%% Desired closed loop
s = tf('s');
bandwidth = 10; % desired bandwidth in Hz
zeta = 1/sqrt(2);
w0 = 2*pi*bandwidth / sqrt(sqrt(4*zeta^4+1) - 2*zeta^2);
T = w0^2 / (s^2 + 2*zeta*w0*s + w0^2);
S = 1-T;

%% Design controller for the front axle
% Evaluate the symbolic matrices to compute the transfer function
mi = mF; ksi = ksF; bsi = bsF;
AF = eval(A_sym);
BF = eval(B_sym);
CF = eval(C_sym);
DF = eval(D_sym);

sys = ss(AF,BF,CF,DF);
GpF = tf(sys);

GpF.InputName{1} = 'tauF';
GpF.OutputName{1} = 'sFx';

% Design controller
YF  = minreal(T/GpF);
GcF = minreal(YF/S);

%% Design controller for the rear axle
% Evaluate the symbolic matrices to compute the transfer function
mi = mR; ksi = ksR; bsi = bsR;
AR = eval(A_sym);
BR = eval(B_sym);
CR = eval(C_sym);
DR = eval(D_sym);

sys = ss(AR,BR,CR,DR);
GpR = tf(sys);

GpR.InputName{1} = 'tauR';
GpR.OutputName{1} = 'sRx';

% Design controller
YR  = minreal(T/GpR);
GcR = minreal(YR/S);

%% Save controllers in a .mat file
save('YoulaSlipControl')



