clear; close all; 
clc;

%% Obtain symbolic Jacobian matrices
run EOM.m

% Define inputs and outputs
u = [viF viR tauf taur];
y = [vF vR sFx sRx];

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
Gp.OutputName{1} = 'vF';
Gp.OutputName{2} = 'vR';
Gp.OutputName{3} = 'sFx';
Gp.OutputName{3} = 'sRx';

% Bode plot
opts = bodeoptions('cstprefs');
opts.FreqUnits = 'Hz';
opts.PhaseVisible = 'off';

figure(1)
opts.Xlim = [1e-1 1e2];
bode(Gp(1:2,1:2),opts,'k')

figure(2)
opts.Xlim = [1e-1 1e5];
opts.Ylim = [-250 -60];
bode(Gp(3:4,3:4),opts,'k')


