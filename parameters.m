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
D = 0.5;                % Pacejka's D coefficient (-)

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

% Parameters used to define suspension bounds
b1 = 3000/0.4;
a1 = 0;
b2 = 1200/0.4;
a2 = 1800;
b3 = 1000/0.6;
a3 = 100;
b4 = 1000/0.6;
a4 = -100;
b5 = 1200/0.4;
a5 = -1800;

%% Road parameters
% Deterministic road parameters
z0_road = 0.02;                 % Amplitude of the road profile (m)
lambda_road = 50/3.6;           % Wavelength of the road profile (m)
w_road = 2*2*pi/lambda_road;    % Frequency of the road profile
d_bump = 5;                     % Distance of the bump to the start (m)

% Random road parameters
G = 1.7e-5;         % Roughness parameter
p = 1.55;           % Waviness
N = 500;            % Number of frequencies
Lmin = 1/15;        % Smallest wavelength considered
Lmax = 60;          % Biggest wavelength considered
nmin = 2*pi/Lmax;   % Smallest wavenumber
nmax = 2*pi/Lmin;   % Biggest wavenumber
Dn = (nmax - nmin)/N;
n_road = nmin:Dn:nmax;   % Wavenumber
S = G * n_road.^(-p);    % PSD
P = 2 * S.*(n_road>=0);  % One side PSD
A_road = sqrt(2*P*Dn);   % Amplitude
Phase_road = 2*pi*rand(size(A_road));  % Phase
