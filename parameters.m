g = 9.81;

%% Chassis parameters
m = 1370;       % Vehicle mass (kg)
a = 1.08;       % Distance from front axle to CG (m)
b = 1.35;       % Distance from rear axle to CG (m)
J = .7*m*a*b;   % Moment of inertia of the chassis around the y axis (kg.m^2)

%% Suspension parameters
wsF = 2*pi*1;           % Front suspension natural frequency (rad/s)
wsR = 2*pi*1.2;         % Rear suspension natural frequency (rad/s)
mF = b/(a+b)*m;         % Mass on the front axle (kg)
mR = a/(a+b)*m;         % Mass on the rear axle (kg)
ksF = mF*wsF^2;         % Front suspension stiffness (N/m)
ksR = mR*wsR^2;         % Rear suspension stiffness (N/m)
zeta_s = .4;            % Suspension dampring ratio (-)
bsF = 2*zeta_s*wsF*mF;  % Front suspension damping (N/ms)
bsR = 2*zeta_s*wsR*mR;  % Rear suspension damping (N/ms)
zeta_t = 0;             % Tire damping ratio (-)
mus = 90;               % Mass of the unsprung mass (kg)
wt = 2*pi*10;           % Wheelhop frequency (rad/s)
kt = mus*wt^2;          % Tire normal stiffness (N/m)
bt = 2*zeta_t*mus*wt;   % Tire normal damping (N/ms)
h_susp = 0.2;           % Nominal length of loaded suspension (m)

% ksF = 2*153e3;          % Carsim values
% ksR = 2*82e3;
% kt = 2*268e3;

%% Tire parameters
Jw = 0.9;               % Moment of inertia of the wheel (kg.m^2)
rw = 0.3;               % Nominal wheel radius with load (m)
B = 7.10;               % Pacejka's B coefficient (-)
C = 1.4;                % Pacejka's C coefficient (-)
D = 1;                  % Pacejka's D coefficient (-)

%% Brake parameters
mcyl = 0.75;            % Piston mass (kg)
dcyl = 0.057;           % Piston diameter (m)
bcyl = 2500;            % Damping coefficient (Nm/s)
V0_brake = 32e-6;       % Nominal chamber volume (m^3)
dline = 0.003;          % Brake line diameter (m)
lline = 2;              % Brake line length (m)
vhf = 30e-6;            % Hydraulic fluid kinematic viscosity (m^2/s)
phf = 1040;             % Hydraulic fluid density (kg/m^3)
bhf = 1.7e9;            % Hydraulic fluid bulk modulus (N/m^2)
x0 = 0.00015;           % Brake clearance (m)
mucal = 0.4;            % Friction coefficient of the caliper (-)
reff = 0.112;           % Effective radius of the brake (m)
kcal = 3.35e7;          % Brake pad stiffness (N/m)
SAcyl = pi*dcyl^2/4;    % Surface area of the piston
Icyl = mcyl;                                % Inertia of the cylinder
Rcyl = bcyl;                                % Resistance of the cylinder
Ccyl = V0_brake/bhf;                        % Compliance of the cylinder
Ccal = 1/kcal;                              % Compliance of the caliper
Iline = phf*lline/(pi*dline^2/4);           % Fluid inertia of the line
Rline = 128*phf*vhf*lline/(pi*dline^4);     % Resistance of the line

%% Semi-active suspension
zetac = 0*0.7;
omegasF = 2*pi*1;
omegasR = 2*pi*1.2;
bcF = 2*zetac*omegasF*mF;
bcR = 2*zetac*omegasR*mR;
