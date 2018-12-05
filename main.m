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
roadProfile_name = 'random';%'sinusoidal';
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

%% ABS parameters
sx_ref = -1/B*tan(pi/(2*C)); % Slip target when ABS is engaged
sxABSon_F = 1.4*sx_ref;
sxABSon_R = 1.4*sx_ref;
sxABSoff  = 0.1*sx_ref;

%% Define parameters of the MPC
param_F = struct('ks',ksF,'bs',bsF,'msp',mF,'kt',kt,'mus',mus,'rw',rw,...
    'B',B,'C',C,'D',D,'Jw',Jw,'g',g,'a1',a1,'a2',a2,'a3',a3,'a4',a4,...
    'a5',a5,'b1',b1,'b2',b2,'b3',b3,'b4',b4,'b5',b5);

param_R = struct('ks',ksR,'bs',bsR,'msp',mR,'kt',kt,'mus',mus,'rw',rw,...
    'B',B,'C',C,'D',D,'Jw',Jw,'g',g,'a1',a1,'a2',a2,'a3',a3,'a4',a4,...
    'a5',a5,'b1',b1,'b2',b2,'b3',b3,'b4',b4,'b5',b5);

% Set MPC parameters
Ts_MPC = 0.05;  % MPC sampling time
Nt_MPC = 10;    % MPC horizon
Nx_MPC = 5;     % Number of states of the MPC model
Nu_MPC = 3;     % Number of inputs of MPC model
Nr_MPC = 4;     % Number of reference signals of the MPC

set_param('model_MPC/MPC Front/MPC',...
    'Delta' ,'Ts_MPC',...
    'Nx'    ,'Nx_MPC',...
    'Nu'    ,'Nu_MPC',...
    'Nr'    ,'Nr_MPC',...
    'Nt'    ,'Nt_MPC',...
    'sx_ref','sx_ref',...
    'param' ,'param_F')

set_param('model_MPC/MPC Rear/MPC',...
    'Delta' ,'Ts_MPC',...
    'Nx'    ,'Nx_MPC',...
    'Nu'    ,'Nu_MPC',...
    'Nr'    ,'Nr_MPC',...
    'Nt'    ,'Nt_MPC',...
    'sx_ref','sx_ref',...
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

set_param('model_MPC/Vehicle/Road preview',...
    'Nt_MPC','Nt_MPC',...
    'Ts_MPC','Ts_MPC',...
    'param' ,'param_road')

%% Load Youla controller for the base model
Gc = load('ControllerBaseModel/YoulaSlipControl.mat','GcF','GcR');
GcF = Gc.GcF;
GcR = Gc.GcR;

% Remove integrator from controller
GcF_noInt = minreal(tf('s')*GcF);
GcR_noInt = minreal(tf('s')*GcR);

%% Simulation parameters
tfinal = 10;    % Duration of the simulation (s)
torque_request = -3000; % Driver's torque request (Nm)

vehicle_MPC   = timeseries();
control_MPC   = timeseries();
vehicle_Youla = timeseries();
control_Youla = timeseries();

U_cutoff = 10/3.6;  % Stop simulation when the vehicle 
                    % speed is below this value

%% Run simulation (MPC tuning)
% Nt_MPC = 50;
% Ts_MPC_list = [0.0250 0.05 0.1 0.15 0.2];
% for k = 1:length(Ts_MPC_list)
%     Ts_MPC = Ts_MPC_list(k);
%     disp(['Run MPC model with Ts = ',num2str(Ts_MPC),...
%         's and Nt = ',num2str(Nt_MPC)])
%     sim('model_MPC.slx')
%     save(['ResultsTuningMPC/Ts_',num2str(k)])
% end
% 
% Ts_MPC = 0.05;
% Nt_MPC_list = [10 20 30 40 50];
% for k = 1:length(Nt_MPC_list)
%     Nt_MPC = Nt_MPC_list(k);
%     disp(['Run MPC model with Ts = ',num2str(Ts_MPC),...
%         's and Nt = ',num2str(Nt_MPC)])
%     sim('model_MPC.slx')
%     save(['ResultsTuningMPC/Nt_',num2str(k)])
% end

%% Run simulation (comparison between Youla and MPC)
% Simulation with MPC control
sim('model_MPC.slx')

% Simulation with Youla
sim('model_base.slx')

% Plot results
plotFigure

%% Plot animation
fps = 6;
frame = drawHalfCar(vehicle_MPC,fps,a,b,rw/3,(a+b)/5,h_susp,rw,z0_road,...
    lambda_road,w_road,d_bump,mF,mR,mus,g,kt,A_road,Phase_road,n_road,...
    roadProfile_index);
% movie(frame,1,fps)
% 
% frame = drawHalfCar(vehicle_Youla,fps,a,b,rw/3,(a+b)/5,h_susp,rw,z0_road,...
%     lambda_road,w_road,d_bump,mF,mR,mus,g,kt,A_road,Phase_road,n_road,...
%     roadProfile_index);
% movie(frame,1,fps)





