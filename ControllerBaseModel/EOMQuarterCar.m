% Provide equation of motion of the quarter car model

%% Define symbolic variables
% Parameters
syms mi ksi bsi mus kt U B C D g rw Jw

% State
% symbolic variables
syms pmi qsi pusi qti sxi

% Inputs
syms Fci taui vii

%% Define velocity of the sprung mass
vi = pmi/mi;

%% Compute suspension and tire forces
% Suspension vertical force (gauge)
fsi = Fci + qsi*ksi + bsi*(pusi/mus - pmi/mi);

% Tire vertical force
fti = qti*kt;

%% Compute longitudinal tire force 
% Compute friction coefficient (simplified Pacejka tire model)
mui = D * sin(C * atan(B * sxi));

% Compute normal force
fiz = fti + (mus + mi)*g;

% Compute longitudinal force (friction force = miu*Fn)
fix = mui * fiz;

%% Compute the state derivatives
% Vertical dynamics
pmi_dot = fsi;
pusi_dot = fti - fsi;

qsi_dot = (pusi/mus) - pmi/mi;
qti_dot = vii - (pusi/mus);

% Longitudinal dynamics
sxi_dot = (rw/(Jw*U)) * ( taui - rw*fix*...
    (1 + (Jw/(mi*rw^2))*(1+sxi)));

%% Define states and state derivatives
state_dot = [pmi_dot; pusi_dot; qsi_dot; qti_dot; sxi_dot];
state = [pmi; pusi; qsi; qti; sxi];





