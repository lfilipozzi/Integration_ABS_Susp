% Plot results for main file

%% Show road profile
% figure(1)
% clf
% subplot(2,1,1)
% plot(x_road,z_road)
% ylabel('z_{road} (m)')
% subplot(2,1,2)
% plot(x_road,DzDx_road)
% ylabel('dz/dx_{road} (m/s)')
% xlabel('x_{road} (m)')
% 
%% Vehicle and wheel speed
% % Vehicle speed
% figure(2)
% hold on
% box on
% plot(vehicle_MPC.U)
% plot(rw*vehicle_MPC.wF)
% plot(rw*vehicle_MPC.wR)
% 
%% Show constraints on SAS (MPC)
% % plot constraints on SAS
% figure(3)
% hold on
% box on
% % Resample qsF and qsR
% qsF_dot_resampled = resample(vehicle_MPC.qsF_dot, control_MPC.FcF.Time);
% qsR_dot_resampled = resample(vehicle_MPC.qsR_dot, control_MPC.FcR.Time);
% % Define xlim of the plot
% vmax = max([.6;
%     abs(qsF_dot_resampled.Data);
%     abs(qsR_dot_resampled.Data)]);
% vmin = -vmax;
% % Define the SAS actuator bounds
% v = [(a1-a3)/(b3-b1); vmax; vmax; (a1-a2)/(b2-b1); 
%     (a1-a5)/(b5-b1); vmin; vmin; (a1-a4)/(b4-b1)];
% F = [b1*v(1)+a1; b3*v(2)+a3; b2*v(3)+a2; b1*v(4)+a1;
%     b1*v(5)+a1; b5*v(6)+a5; b4*v(7)+a4; b1*v(8)+a1];
% polytope = fill(v,F,'k');
% set(polytope,'FaceColor',0.95*[1 1 1],...
%     'EdgeColor','k')
% % Plot each point given by the MPC
% SAS_bounds_F = plot(qsF_dot_resampled.Data, control_MPC.FcF.Data, '.');
% SAS_bounds_R = plot(qsR_dot_resampled.Data, control_MPC.FcR.Data, '.');
% 
% xlabel('Suspension relative velocity $v = v_{sprung} - v_{unsprung}$ (m/s)')
% ylabel('Suspension force $F_c$ (N)')
% myLeg = legend([SAS_bounds_F SAS_bounds_R],'Front','Rear');
% 
% set(myLeg,'location','northwest')
% set(gca,'Layer','top')

%% Show constraints on SAS (Youla)
% figure(4)
% hold on
% box on
% % Define xlim of the plot
% vmax = max([.6;
%     abs(vehicle_Youla.qsF_dot.Data);
%     abs(vehicle_Youla.qsR_dot.Data)]);
% vmin = -vmax;
% % Define the SAS actuator bounds
% v = [(a1-a3)/(b3-b1); vmax; vmax; (a1-a2)/(b2-b1); 
%     (a1-a5)/(b5-b1); vmin; vmin; (a1-a4)/(b4-b1)];
% F = [b1*v(1)+a1; b3*v(2)+a3; b2*v(3)+a2; b1*v(4)+a1;
%     b1*v(5)+a1; b5*v(6)+a5; b4*v(7)+a4; b1*v(8)+a1];
% polytope = fill(v,F,'k');
% set(polytope,'FaceColor',0.95*[1 1 1],...
%     'EdgeColor','k')
% % Plot each point given by the MPC
% SAS_bounds_F = plot(vehicle_Youla.qsF_dot.Data, control_Youla.FcF.Data,'.');
% SAS_bounds_R = plot(vehicle_Youla.qsR_dot.Data, control_Youla.FcR.Data,'.');
% 
% xlabel('Suspension relative velocity $v = v_{sprung} - v_{unsprung}$ (m/s)')
% ylabel('Suspension force $F_c$ (N)')
% myLeg = legend([SAS_bounds_F SAS_bounds_R],'Front','Rear');
% 
% set(myLeg,'location','northwest')
% set(gca,'Layer','top')


