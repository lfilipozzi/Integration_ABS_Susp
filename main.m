clear; clc;
close all;

% Add path to CasADi and MPCTools folder
addpath('/Echange/Ecole/17 - UC Davis/MAE298-Optimization Based Control/MPCTools/casadi/',...
    '/Echange/Ecole/17 - UC Davis/MAE298-Optimization Based Control/MPCTools/mpctools/')

% addpath('D:/Ecole/17 - UC Davis/MAE298-Optimization Based Control/MPCTools/casadi_win/',...
%     'D:/Ecole/17 - UC Davis/MAE298-Optimization Based Control/MPCTools/mpctools/')

%% Define vehicle and road parameters
parameters

%% Define sinusoidal road profile
% Choose road profile. Possible values: '
roadProfile_name = 'sinusoidal';
roadProfile_index = getRoadIndex(roadProfile_name);

% Plot road profile
x_road = linspace(0,100,1000);
z_road = zeros(size(x_road));
DzDx_road = zeros(size(x_road));
for i = 1:length(x_road)
    [DzDx_road(i),z_road(i)] = ...
        roadProfile(x_road(i),z0_road,lambda_road,w_road,d_bump,...
        A_road,Phase_road,n_road,roadProfile_index);
end

% figure(1)
% clf
% subplot(2,1,1)
% plot(x_road,z_road)
% ylabel('z_{road} (m)')
% subplot(2,1,2)
% plot(x_road,DzDx_road)
% ylabel('dz/dx_{road} (m/s)')
% xlabel('x_{road} (m)')

%% Define initial state
U_0 = 100/3.6;
disp(['Initial velocity: ',num2str(3.6*U_0),' km/h'])

% Vehicle state
state_init = [0;    % pm
    0;          % pJ 
    0;          % pmF
    0;          % pmR
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
param_F = struct();
param_F.ks  = ksF;
param_F.bs  = bsF;
param_F.kt  = kt;
param_F.mus = mus;
param_F.rw  = rw;
param_F.B   = B;
param_F.C   = C;
param_F.D   = D;
param_F.Jw  = Jw;
param_F.g   = g;
param_F.msp = mF;
param_F.a1  = a1;
param_F.a2  = a2;
param_F.a3  = a3;
param_F.a4  = a4;
param_F.a5  = a5;
param_F.b1  = b1;
param_F.b2  = b2;
param_F.b3  = b3;
param_F.b4  = b4;
param_F.b5  = b5;

param_R = struct();
param_R.ks  = ksR;
param_R.bs  = bsR;
param_R.kt  = kt;
param_R.mus = mus;
param_R.rw  = rw;
param_R.B   = B;
param_R.C   = C;
param_R.D   = D;
param_R.Jw  = Jw;
param_R.g   = g;
param_R.msp = mR;
param_R.a1  = a1;
param_R.a2  = a2;
param_R.a3  = a3;
param_R.a4  = a4;
param_R.a5  = a5;
param_R.b1  = b1;
param_R.b2  = b2;
param_R.b3  = b3;
param_R.b4  = b4;
param_R.b5  = b5;

%% ABS parameters
sx_ref = -1/B*tan(pi/(2*C)); % Slip target when ABS is engaged
sxABSon_F = 1.4*sx_ref;
sxABSon_R = 1.4*sx_ref;
sxABSoff  = 0.2*sx_ref;

%% Set MPC parameters
Ts_MPC = 0.025; % MPC sampling time
Nt_MPC = 40;    % MPC horizon
Nx_MPC = 5;     % Number of states of the MPC model
Nu_MPC = 3;     % Number of inputs of MPC model
Nr_MPC = 4;     % Number of reference signals of the MPC

set_param('model/MPC Front/MPC',...
    'Delta' ,num2str(Ts_MPC),...
    'Nx'    ,num2str(Nx_MPC),...
    'Nu'    ,num2str(Nu_MPC),...
    'Nr'    ,num2str(Nr_MPC),...
    'Nt'    ,num2str(Nt_MPC),...
    'sx_ref',num2str(sx_ref),...
    'param' ,'param_F')

set_param('model/MPC Rear/MPC',...
    'Delta' ,num2str(Ts_MPC),...
    'Nx'    ,num2str(Nx_MPC),...
    'Nu'    ,num2str(Nu_MPC),...
    'Nr'    ,num2str(Nr_MPC),...
    'Nt'    ,num2str(Nt_MPC),...
    'sx_ref',num2str(sx_ref),...
    'param' ,'param_R')

% Need to use MATLAB System for road preview since MATLAB thinks the size
% of the matrix is changing over time but this is wrong
param_road = struct();
param_road.a = a;
param_road.b = b;
param_road.z0_road = z0_road;
param_road.w_road = w_road;
param_road.lambda_road = lambda_road;
param_road.d_bump = d_bump;
param_road.A_road = A_road;
param_road.Phase_road = Phase_road;
param_road.n_road = n_road;
param_road.roadProfile_index = roadProfile_index;

set_param('model/Vehicle/Road preview',...
    'Nt_MPC',num2str(Nt_MPC),...
    'Ts_MPC',num2str(Ts_MPC),...
    'param' ,'param_road')

%% Run simulation
vehicle = timeseries();
tfinal = 10;    % Duration of the simulation (s)
sim('model.slx')

%% Plot
% % % Plot theta
% % figure(2)
% % plot(vehicle.theta*180/pi)
% % ylabel('Theta (deg)')
% % title('Theta vs time')
% 
% % Plot longitudinal force and slip vs time and force vs slip
% figure(3)
% % Force vs time
% subplot(2,2,1)
% hold on
% box on
% plot(vehicle.fFx)
% plot(vehicle.fRx)
% legend('Front','Rear')
% ylabel('Longitudinal force (N)')
% title('')
% % Slip vs time
% subplot(2,2,3)
% hold on
% box on
% plot(vehicle.sFx)
% plot(vehicle.sRx)
% ylabel('Longitudinal slip (-)')
% title('')
% % Force vs slip
% subplot(2,2,[2 4])
% hold on
% box on
% plot(vehicle.sFx.Data, vehicle.fFx.Data./vehicle.fFz.Data,'o')
% plot(vehicle.sRx.Data, vehicle.fRx.Data./vehicle.fRz.Data,'o')
% xlabel('Longidutinal slip (-)')
% ylabel('Longitudinal friction coef (-)')
% title('')
% 
% % Plot vertical force vs time
% figure(4)
% hold on
% box on
% plot(vehicle.fFz)
% plot(vehicle.fRz)
% legend('Front','Rear')
% xlabel('Time (s)')
% ylabel('Vertical force (N)')
% title('Vertical force vs time')
% 
% % % Suspension and tire displacement
% % figure(5)
% % % Suspension displacement
% % subplot(2,1,1)
% % hold on
% % box on
% % plot(vehicle.qsF)
% % plot(vehicle.qsR)
% % legend('Front','Rear')
% % ylabel({'Suspension';'displacement'})
% % % Tire displacement
% % subplot(2,1,2)
% % hold on
% % box on
% % plot(vehicle.qtF)
% % plot(vehicle.qtR)
% % ylabel({'Tire';'displacement'})
% % xlabel('Time (s)')
% 
% % % Heave
% % figure(6)
% % plot(vehicle.h)
% 
% % Vehicle speed
% figure(7)
% hold on
% box on
% plot(vehicle.U)
% plot(rw*vehicle.wF)
% plot(rw*vehicle.wR)

% plot constraints on SAS
figure(8)
hold on
box on
% Resample qsF and qsR
qsF_dot_resampled = resample(vehicle.qsF_dot, mpc_results.FcF.Time);
qsR_dot_resampled = resample(vehicle.qsR_dot, mpc_results.FcR.Time);
% Define xlim of the plot
vmax = max([.6;
    abs(qsF_dot_resampled.Data);
    abs(qsR_dot_resampled.Data)]);
vmin = -vmax;
% Define the SAS actuator bounds
v = [(a1-a3)/(b3-b1); vmax; vmax; (a1-a2)/(b2-b1); 
    (a1-a5)/(b5-b1); vmin; vmin; (a1-a4)/(b4-b1)];
F = [b1*v(1)+a1; b3*v(2)+a3; b2*v(3)+a2; b1*v(4)+a1;
    b1*v(5)+a1; b5*v(6)+a5; b4*v(7)+a4; b1*v(8)+a1];
polytope = fill(v,F,'k');
set(polytope,'FaceColor',0.95*[1 1 1],...
    'EdgeColor','k')
% Plot each point given by the MPC
SAS_bounds_F = plot(qsF_dot_resampled.Data, mpc_results.FcF.Data, '.');
SAS_bounds_R = plot(qsR_dot_resampled.Data, mpc_results.FcR.Data, '.');

xlabel('Suspension relative velocity $v = v_{sprung} - v_{unsprung}$ (m/s)')
ylabel('Suspension force $F_c$ (N)')
myLeg = legend([SAS_bounds_F SAS_bounds_R],'Front','Rear');

set(myLeg,'location','northwest')
set(gca,'Layer','top')

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
%     lambda_road,w_road,d_bump,mF,mR,mus,g,kt,A_road,Phase_road,n_road,...
%     roadProfile_index);
% % movie(frame,1,fps)





