% Plot results for main file

close all;

% plotName = 'sinusoidal';    % Name of the plots
folder = 'ResultsRMPC';
plotName = 'random';        % Name of the plots
publishFigure = false;      % Switch to save figure as pdf file

%% Change graphics parameters
FontSize = 7;
set(groot,'defaultTextInterpreter','latex')
set(groot,'defaultAxesTickLabelInterpreter','latex')
set(groot,'defaultLegendInterpreter','latex')
set(groot,'Units','centimeters')
set(groot,'defaultFigureUnits','centimeters')
set(groot,'defaultAxesUnits','normalized')
set(groot,'defaultAxesFontSize',FontSize)
set(groot,'defaultFigureColor',[1 1 1])

%% Show road profile
% figure(1)
% clf
% subplot(2,1,1)
% plot(x_road,z_road)
% ylabel('$z_{road}$ (m)')
% subplot(2,1,2)
% plot(x_road,DzDx_road)
% ylabel('$dz/dx_{road}$ (m/s)')
% xlabel('$x_{road}$ (m)')
% 
% set(gcf,'pos',[20 25 12 8])
% 
% if publishFigure
%     pause(1)
%     export_fig([folder,'/',plotName,'_road.pdf'])
% end

%% Vehicle speed
% Vehicle speed
figure(2)
hold on
box on
plot(vehicle_MPC.U)
plot(vehicle_RMPC.U)

time_max = max([vehicle_MPC.U.Time;
    vehicle_RMPC.U.Time]);

xlim([0 time_max])
xlabel('Time (s)')
ylabel('Vehicle velocity (m/s)')

hleg = legend('MPC','min-max MPC');
set(hleg,'location','southwest')

set(gcf,'pos',[10 5 6 6])

if publishFigure
    pause(1)
    export_fig([folder,'/',plotName,'_velocity.pdf'])
end

%% Show constraints on SAS (MPC)
% plot constraints on SAS
figure(3)
hold on
box on
% Resample qsF and qsR
qsF_dot_resampled = resample(vehicle_MPC.qsF_dot, control_MPC.FcF.Time);
qsR_dot_resampled = resample(vehicle_MPC.qsR_dot, control_MPC.FcR.Time);
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
SAS_bounds_F = plot(qsF_dot_resampled.Data, control_MPC.FcF.Data, '.');
SAS_bounds_R = plot(qsR_dot_resampled.Data, control_MPC.FcR.Data, '.');

xlim([vmin vmax])

xlabel('Suspension relative velocity $v = v_{sprung} - v_{unsprung}$ (m/s)')
ylabel('Suspension force $F_c$ (N)')
myLeg = legend([SAS_bounds_F SAS_bounds_R],'Front','Rear');

set(myLeg,'location','northwest')
set(gca,'Layer','top')

set(gcf,'pos',[2 5 8 6])

if publishFigure
    pause(1)
    export_fig([folder,'/',plotName,'_SASConstraint_MPC.pdf'])
end

%% Show constraints on SAS (min-max MPC)
% plot constraints on SAS
figure(4)
hold on
box on
% Resample qsF and qsR
qsF_dot_resampled = resample(vehicle_RMPC.qsF_dot, control_RMPC.FcF.Time);
qsR_dot_resampled = resample(vehicle_RMPC.qsR_dot, control_RMPC.FcR.Time);
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
SAS_bounds_F = plot(qsF_dot_resampled.Data, control_RMPC.FcF.Data, '.');
SAS_bounds_R = plot(qsR_dot_resampled.Data, control_RMPC.FcR.Data, '.');

xlim([vmin vmax])

xlabel('Suspension relative velocity $v = v_{sprung} - v_{unsprung}$ (m/s)')
ylabel('Suspension force $F_c$ (N)')
myLeg = legend([SAS_bounds_F SAS_bounds_R],'Front','Rear');

set(myLeg,'location','northwest')
set(gca,'Layer','top')

set(gcf,'pos',[2 5 8 6])

if publishFigure
    pause(1)
    export_fig([folder,'/',plotName,'_SASConstraint_minmaxMPC.pdf'])
end

%% Comparison slip RMPC
figure(5)
sx_min = min([vehicle_MPC.sFx.Data;
    vehicle_RMPC.sFx.Data;
    vehicle_MPC.sRx.Data;
    vehicle_RMPC.sRx.Data]);
sx_max = 1.2*max([vehicle_MPC.sFx.Data;
    vehicle_RMPC.sFx.Data
    vehicle_MPC.sRx.Data;
    vehicle_RMPC.sRx.Data]);
time_max = max([vehicle_MPC.sFx.Time;
    vehicle_RMPC.sFx.Time]);

% Front
subplot(1,2,1)
hold on
box on
plot(vehicle_MPC.sFx)
plot(vehicle_RMPC.sFx)

xlim([0 time_max])
ylim([sx_min sx_max])

xlabel('Time (s)')
ylabel('Longitudinal slip $s_x$')
title('Front axle')

set(gca,'position',[0.13 0.15 0.4 0.75])

% Rear
subplot(1,2,2)
hold on
box on
plot(vehicle_MPC.sRx)
plot(vehicle_RMPC.sRx)

xlim([0 time_max])
ylim([sx_min sx_max])

xlabel('Time (s)')
title('Rear axle')
hleg = legend('MPC','min-max MPC');
set(hleg,'location','southeast')

set(gca,'YTick',[])
set(gca,'position',[0.56 0.15 0.4 0.75])

set(gcf,'pos',[2 15 12 5])

if publishFigure
    pause(1)
    export_fig([folder,'/',plotName,'_sx.pdf'])
end

%% Comparison vertical force MPC and RMPC
figure(6)
fz_min = min([vehicle_MPC.fFz.Data;
    vehicle_RMPC.fFz.Data;
    vehicle_MPC.fRz.Data;
    vehicle_RMPC.fRz.Data]);
fz_max = 1.2*max([vehicle_MPC.fFz.Data;
    vehicle_RMPC.fFz.Data;
    vehicle_MPC.fRz.Data;
    vehicle_RMPC.fRz.Data]);
time_max = max([vehicle_MPC.fFz.Time;
    vehicle_MPC.fFz.Time]);

% Front
subplot(1,2,1)
hold on
box on
plot(vehicle_MPC.fFz)
plot(vehicle_RMPC.fFz)

xlim([0 time_max])
ylim([fz_min fz_max])

xlabel('Time (s)')
ylabel('Vertical tire force $f_z$')
title('Front axle')
hleg = legend('MPC','min-max MPC');
set(hleg,'location','southeast')

set(gca,'position',[0.13 0.15 0.4 0.75])

% Rear
subplot(1,2,2)
hold on
box on
plot(vehicle_MPC.fRz)
plot(vehicle_RMPC.fRz)

xlim([0 time_max])
ylim([fz_min fz_max])

xlabel('Time (s)')
title('Rear axle')

set(gca,'YTick',[])
set(gca,'position',[0.56 0.15 0.4 0.75])

set(gcf,'pos',[2 25 12 5])

if publishFigure
    pause(1)
    export_fig([folder,'/',plotName,'_fz.pdf'])
end

%% Show road profile and low-frequencies road profile used by min-max MPC
figure(7)
clf
subplot(2,1,1)
hold on
box on
plot(x_road,z_road)
plot(x_road,z_road_lowfreq)
ylabel('$z_{road}$ (m)')
subplot(2,1,2)
hold on
box on
plot(x_road,DzDx_road)
plot(x_road,DzDx_road_lowfreq)
ylabel('$dz/dx_{road}$ (m/s)')
xlabel('$x_{road}$ (m)')

set(gcf,'pos',[20 25 12 8])

if publishFigure
    pause(1)
    export_fig([folder,'/',plotName,'_road_withlowfreq.pdf'])
end

%% Reset graphics parameter to default
set(groot,'defaultLineLineWidth',1)
set(groot,'defaultTextInterpreter','tex')
set(groot,'defaultAxesTickLabelInterpreter','tex')
set(groot,'defaultLegendInterpreter','tex')
set(groot,'Units','pixels')
set(groot,'defaultFigureUnits','pixels')
set(groot,'defaultAxesUnits','normalized')
set(groot,'defaultAxesFontSize',10)
set(groot,'defaultFigureColor',[.94 .94 .94])

