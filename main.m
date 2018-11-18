clear; clc;
close all;

% Add path to CasADi and MPCTools folder
addpath('/Echange/Ecole/17 - UC Davis/MAE298-Optimization Based Control/MPCTools/casadi/',...
    '/Echange/Ecole/17 - UC Davis/MAE298-Optimization Based Control/MPCTools/mpctools/')

%% Define parameters
parameters

%% Define sinusoidal road profile
z0_road = 0.02;             % Amplitude of the road profile (m)
lambda_road = .5;           % Wavelength of the road profile (m)
w_road = 2*pi/lambda_road;  % Frequency of the road profile
d_bump = 5;                 % Distance of the bump to the start (m)

% Road PSD parameters
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

% Plot road profile
x_road = linspace(0,100,1000);
z_road = zeros(size(x_road));
DzDx_road = zeros(size(x_road));
for i = 1:length(x_road)
    [DzDx_road(i),z_road(i)] = ...
        roadProfile(x_road(i),z0_road,lambda_road,w_road,d_bump,...
        A_road,Phase_road,n_road);
end

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
U_0 = 60/3.6;
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
    rw+h_susp;  % h
    U_0;        % U
    U_0/rw;     % wF
    U_0/rw;     % wR
    0];         % x

% Brake state
brake_init = [0; 0; 0; 0; x0];

%% Define parameters of the MPC
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
%         a*cos(vehicle.theta.Data(i)),z0_road,lambda_road,w_road,d_bump,...
%         A_road,Phase_road,n_road);
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
% fps = 6;
% frame = drawHalfCar(vehicle,fps,a,b,rw/3,(a+b)/5,h_susp,rw,z0_road,...
%     lambda_road,w_road,d_bump,mF,mR,mus,g,kt,A_road,Phase_road,n_road);
% % movie(frame,1,fps)





