clear; close all; 
clc;

%% Import MPCTools
addpath('/Echange/Ecole/17 - UC Davis/MAE298-Optimization Based Control/MPCTools/casadi/',...
    '/Echange/Ecole/17 - UC Davis/MAE298-Optimization Based Control/MPCTools/mpctools/')

mpc = import_mpctools();

%% Define parameters
run parameters.m
param = struct();
param.m   = m;
param.J   = J;
param.a   = a;
param.b   = b;
param.ksR = ksR;
param.ksF = ksF;
param.bsF = bsF;
param.bsR = bsR;
param.kt  = kt;
param.bt  = bt;
param.mus = mus;
param.rw  = rw;
param.B   = B;
param.C   = C;
param.D   = D;
param.Jw  = Jw;
param.g   = g;
param.mF  = mF;
param.mR  = mR;
param.h_susp = h_susp;

%% Define model
Delta = 0.01;                   % Sampling time
Nx = 14;                        % Number of state
Nu = 4;                         % Number of input
Nt = 20;                        % Prediction horizon

% Define the (non-linear) model
kwargs = struct('funcname', 'ode'); % Name for the Integrator object
Nc = 3;
fnonlin = mpc.getCasadiFunc(@(x,u) EOM(x,u,param), ...
    [Nx, Nu], {'x', 'u'}, '**', kwargs);

% Linearize the model
% TODO: define operating point for linearization instead of ones
linmodel = mpc.getLinearizedModel(fnonlin, ...
    {ones(Nx, 1), ones(Nu, 1)}, {'A', 'B'}, Delta); 

% Define the linearized model (x_dot = A*x+Bu)
Flin = mpc.getCasadiFunc(@(x, u) linmodel.A*x + linmodel.B*u, ...
    [Nx, Nu], {'x', 'u'}, {'dintlin'}); 

%% Define cost functions
% Stage cost function
Q = eye(Nx);
R = eye(Nu);
stagecost = @(x,u) (x'*Q*x + u'*R*u)/2;

% Terminal cost function
P = eye(Nx);
termcost = @(x) (x'*P*x)/2;

% Define CasADi cost functions
l = mpc.getCasadiFunc(stagecost, [Nx, Nu], {'x','u'}, {'l'});
Vf = mpc.getCasadiFunc(termcost, Nx, {'x'}, {'Vf'});

%% Specify bounds.
max_x = Inf;
max_u = Inf;%0.1;

lb = struct();
lb.u = -ones(Nu, Nt)*max_u;
lb.x = -ones(1,Nt+1)*Inf;

ub = struct();
ub.u = ones(Nu, Nt)*max_u;
ub.x = ones(Nx, Nt+1)*max_x;

%% Build solvers.
% Define number of state, input and horizon
N = struct('x', Nx, 'u', Nu, 't', Nt); 
% Define cost function and bounds
kwargs = struct('l', l, 'Vf', Vf, 'lb', lb, 'ub', ub, 'verbosity',3);
% Build MPC solver
solver = mpc.nmpc('f', Flin, 'N', N, '**', kwargs);

%% Solve MPC for given x
x = ones(14,1);
solver.fixvar('x', 1, x);
solver.solve();

% Return error if the problem has not been solved
if ~isequal(solver.status, 'Solve_Succeeded')
    warning('Solver failed!');
end

% Return the first control input
u = solver.var.u(:,1);


