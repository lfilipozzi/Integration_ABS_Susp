clear; clc;
close all;

%% Load damper force vs velocity data from Carsim
data = [ -1410, -5008;
 -720, -3436;
 -390, -2324;
 -210, -1792;
 -90, -1008;
 -20, -228;
 20, 228;
 90, 596;
 200, 784;
 390, 1100;
 760, 1796;
 1160, 2560];

v_rel = data(:,1)*1e-3; % Relative velocity (m/s)
F_damper = 2*data(:,2); % Damper force for one axle (N)
v_rel = -v_rel;   % Displacement positive in compression
F_damper = -F_damper;

% Perform regression
model_order = 1;
F_model = F_damper;
v_model = zeros(length(F_model),model_order + 1);
for i = 1:model_order
    v_model(:,i+1) = v_rel.^i;
end
w = regress(F_model,v_model);

% Compute for plot
v_list = linspace(min(v_rel),max(v_rel),1000)';
v_model_plot = zeros(length(v_list),model_order + 1);
for i = 1:model_order
    v_model_plot(:,i+1) = v_list.^i;
end
F_model_plot = v_model_plot*w;

figure(1)
clf
hold on
box on
title('Damper force vs relative velocity')
plot(v_rel,F_damper)
% plot(v_list,F_model_plot)
plot(v_list,3.8258e+03*v_list)  % zeta = 0.4
plot(v_list,6.6951e+03*v_list)  % zeta = 0.7
xlabel('Relative velocity (m/s)')
ylabel('Damper force (N)')
grid on