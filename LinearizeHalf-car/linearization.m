clear; close all; 
clc;

%% Obtain symbolic Jacobian matrices
run EOM.m

% Define inputs and outputs
u = [viF viR tauf taur];
y = [pm/m pJ/J U_dot];

% Compute the symbolic Jacobian matrices
A_sym = jacobian(dx,x);
B_sym = jacobian(dx,u);
C_sym = jacobian(y,x);
D_sym = jacobian(y,u);

%% Define operating point
U_0 = 10/3.6;
% State operating point
pm = 0;
pJ = 0;
pmF = 0;
pmR = 0;
qsF = 0;
qsR = 0;
qtF = 0;
qtR = 0;
theta = 0;
h = rw+h_susp;
U = U_0;
wF = U_0/rw;
wR = U_0/rw;
x = 0;

% viF viR Fcf Fcr tauf taur

%% Evaluate the symbolic matrices
% Evaluate the symbolic matrices
A = eval(eval(A_sym));
B = eval(B_sym);
C = eval(C_sym);
D = eval(D_sym);

%% Transfer function
sys = ss(A,B,C,D);
Gp = tf(sys);

Gp.InputName{1} = 'viF';
Gp.InputName{2} = 'viR';
Gp.InputName{3} = 'tauF';
Gp.InputName{4} = 'tauR';
Gp.OutputName{1} = 'V';
Gp.OutputName{2} = 'theta_dot';
Gp.OutputName{3} = 'U_dot';

% Bode plot
opts = bodeoptions('cstprefs');
opts.FreqUnits = 'Hz';
opts.PhaseVisible = 'off';
opts.Xlim = [1e-1 1e2];
% opts.Ylim = [-50 10];
bode(Gp,opts)


