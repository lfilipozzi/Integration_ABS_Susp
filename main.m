clear; clc;
close all;

%% Define parameters
g = 9.81;

% Chassis parameters
m = 1370;       % Vehicle mass (kg)
a = 1.08;       % Distance from front axle to CG (m)
b = 1.35;       % Distance from rear axle to CG (m)
J = .7*m*a*b;   % Moment of inertia of the chassis around the y axis (kg.m^2)

% Suspension parameters
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

% Tire parameters
Jw = 0.9;               % Moment of inertia of the wheel (kg.m^2)
rw = 0.3;               % Nominal wheel radius with load (m)
B = 7.10;               % Pacejka's B coefficient (-)
C = 1.4;                % Pacejka's C coefficient (-)
D = 1;                  % Pacejka's D coefficient (-)

% Brake parameters
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

% Semi-active suspension
zetac = 0*0.7;
omegasF = 2*pi*1;
omegasR = 2*pi*1.2;
bcF = 2*zetac*omegasF*mF;
bcR = 2*zetac*omegasR*mR;

%% Define sinusoidal road profile
z0_road = 0.05;             % Amplitude of the road profile (m)
lambda_road = .5;           % Wavelength of the road profile (m)
w_road = 2*pi/lambda_road;  % Frequency of the road profile
d_bump = 5;                 % Distance of the bump to the start (m)

% Plot road profile
x_road = linspace(0,30,1000);
z_road = zeros(size(x_road));
DzDx_road = zeros(size(x_road));
for i = 1:length(x_road)
    [DzDx_road(i),z_road(i)] = ...
        roadProfile(x_road(i),z0_road,lambda_road,w_road,d_bump);
end

% Road profile
figure(1)
clf
subplot(2,1,1)
plot(x_road,z_road)
ylabel('z_{road} (m)')
subplot(2,1,2)
plot(x_road,DzDx_road)
ylabel('dz/dx_{road} (m/s)')
xlabel('x_{road} (m)')

%% Define initial state
U_0 = 10/3.6;
disp(['Initial velocity: ',num2str(3.6*U_0),' km/h'])

% Vehicle state
state_init = [0;    % pm
    0;          % pJ 
    0;          % pmR
    0;          % pmF
    0;          % qsF
    0;          % qsR
    0;          % qtF
    0;          % qtR
    0;          % theta
    rw+h_susp;  %h
    U_0;        % U
    U_0/rw;     % wF
    U_0/rw;     % wR
    0];         % x

% Brake state
brake_init = [0; 0; 0; 0; x0];

%% Run simulation
tfinal = 10;    % Duration of the simulation (s)
sim('model.slx')

%% Plot
% Plot theta
figure(2)
plot(vehicle.theta*180/pi)
ylabel('Theta (deg)')
title('Theta vs time')

% Plot longitudinal force and slip vs time and force vs slip
figure(3)
% Force vs time
subplot(2,2,1)
hold on
box on
plot(vehicle.fFx)
plot(vehicle.fRx)
legend('Front','Rear')
ylabel('Longitudinal force (N)')
title('')
% Slip vs time
subplot(2,2,3)
hold on
box on
plot(vehicle.sFx)
plot(vehicle.sRx)
ylabel('Longitudinal slip (-)')
title('')
% Force vs slip
subplot(2,2,[2 4])
hold on
box on
plot(vehicle.sFx.Data, vehicle.fFx.Data./vehicle.fFz.Data,'o')
plot(vehicle.sRx.Data, vehicle.fRx.Data./vehicle.fRz.Data,'o')
xlabel('Longidutinal slip (-)')
ylabel('Longitudinal friction coef (-)')
title('')

% Plot vertical force vs time
figure(4)
hold on
box on
plot(vehicle.fFz)
plot(vehicle.fRz)
legend('Front','Rear')
xlabel('Time (s)')
ylabel('Vertical force (N)')
title('Vertical force vs time')

% Suspension and tire displacement
figure(5)
% Suspension displacement
subplot(2,1,1)
hold on
box on
plot(vehicle.qsF)
plot(vehicle.qsR)
legend('Front','Rear')
ylabel({'Suspension';'displacement'})
% Tire displacement
subplot(2,1,2)
hold on
box on
plot(vehicle.qtF)
plot(vehicle.qtR)
ylabel({'Tire';'displacement'})
xlabel('Time (s)')

% Heave
figure(6)
plot(vehicle.h)

%% Plot position of wheels
% z_tire_F = zeros(size(vehicle.x.Data));
% rwF_unladen = rw + (mF+mus)*g/kt;
% for i = 1:length(z_tire_F)
%     [~,road_input] = roadProfile(vehicle.x.Data(i) + ...
%         a*cos(vehicle.theta.Data(i)),z0_road,lambda_road,w_road,d_bump);
%     rwF = min(rw - vehicle.qtF.Data(i), rwF_unladen);
%     z_tire_F(i) = vehicle.h.Data(i) + a*sin(vehicle.theta.Data(i)) ...
%         - (h_susp - vehicle.qsF.Data(i)) - rwF...
%         - road_input;
% end
% 
% figure(7)
% clf
% hold on
% box on
% plot(vehicle.x.time,z_tire_F)

%% Plot animation
fps = 6;
frame = drawHalfCar(vehicle,fps,a,b,rw/3,(a+b)/5,h_susp,rw,z0_road,...
    lambda_road,w_road,d_bump,mF,mR,mus,g,kt);
% movie(frame,1,fps)





