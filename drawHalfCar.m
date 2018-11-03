function frame = drawHalfCar(vehicle,fps,a,b,mass_thick,unsprung_length,...
    h_susp,rw,z0_road,lambda_road,w_road,d_bump,mF,mR,mus,g,kt,...
    A_road,Phase_road,n_road)
%DRAWHALFCAR creates an animation of the 

% This function create an animation of the halfcar model on a given road
% profile. Inputs are:
%   vehicle: structure: which gather all the information on the vehicle.
%       vehicle.x: timeseries: x coordinate of the sprung mass CG
%       vehicle.h: timeseries: z coordinate of the sprung mass CG
%       vehicle.theta: timeseries: pitch angle of the sprung mass
%       vehicle.qsF: timeseries: displacement of the front suspension 
%       vehicle.qsR: timeseries: displacement of the rear suspension 
%       vehicle.qtF: timeseries: displacement of the front tire 
%       vehicle.qtR: timeseries: displacement of the rear tire 
%   a: scalar: distance from front axle to CG
%   b: scalar: distance from rear axle to CG
%   mass_thick: scalar: length along z-axis of the masses
%   unsprung_length: scalar: length along x-axis of unpsrung mass
%   h_susp: scalar: height of the suspension under load
%   rw: scalar: height of the wheel under load
% 
% x-axis is pointing in the forward direction of the vehicle and z-axis is
% pointing upward. The frame is attached to the ground. Displacement are 
% set to be positive in compression. Units must be SI units. Angle theta is
% set to be positive for rotation from x-axis to z-axis.

z_mass = mass_thick;
x_tire = unsprung_length;

nb_coil = 3;    % Number of coil in the spring

rwF_unladen = rw + (mF+mus)*g/kt;   % Unladen front wheel radius
rwR_unladen = rw + (mR+mus)*g/kt;   % Unladen rear wheel radius

%% Resample signal to correspond to the desired fps
% Get length of animation
tfinal = max(vehicle.x.Time);
tinit  = min(vehicle.x.time);
movie_length = tfinal - tinit;
% Set sampling time to correspond to the desired fps
ts = 0:1/(fps*movie_length):tfinal;
% Resample required signals
vehicle.x     = resample(vehicle.x,    ts);
vehicle.h     = resample(vehicle.h,    ts);
vehicle.theta = resample(vehicle.theta,ts);
vehicle.qsF   = resample(vehicle.qsF,  ts);
vehicle.qsR   = resample(vehicle.qsR,  ts);
vehicle.qtF   = resample(vehicle.qtF,  ts);
vehicle.qtR   = resample(vehicle.qtR,  ts);
% Rename to avoid using timeseries
x     = vehicle.x.Data;
h     = vehicle.h.Data;
theta = vehicle.theta.Data;
qsF   = vehicle.qsF.Data;
qsR   = vehicle.qsR.Data;
qtF   = vehicle.qtF.Data;
qtR   = vehicle.qtR.Data;

%% Initialize picture
figure('Name','Half-car animation','numbertitle','off')
hold on
box on

% Draw road
x_road = linspace(min(x)-2*b, max(x)+2*a,10000);
z_road = zeros(size(x_road));
for i = 1:numel(z_road)
    [~,z_road(i)] = roadProfile(x_road(i), z0_road, lambda_road, w_road, ...
        d_bump, A_road, Phase_road, n_road);
end
plot(x_road,z_road,'k');

% Initialize line plot for springs
spring_sF = plot(0,0,'k');
spring_sR = plot(0,0,'k');
spring_tF = plot(0,0,'k');
spring_tR = plot(0,0,'k');

% Initialize rectangle for masses
unsprungF_mass = rectangle();
unsprungR_mass = rectangle();
sprung_mass = patch([0 0 1 1 0],[0 1 1 0 0],[1 1 1]);

%% Create animation
frame(numel(ts)) = struct('cdata',[],'colormap',[]);

for i = 1:numel(ts)
    %% Draw suspension and tire springs
    x_F = x(i) + a*cos(theta(i));   % x coordinate of front axle
    x_R = x(i) - b*cos(theta(i));   % x coordinate of rear axle
    
    rwF = min(rw - qtF(i), rwF_unladen);   % Front tire radius
    rwR = min(rw - qtR(i), rwR_unladen);   % Rear tire radius

    % Suspension spring
    z_up_sF  = h(i) + a*sin(theta(i));
    z_up_tF = z_up_sF - (h_susp - qsF(i));
    [x_spring_sF,y_spring_sF] = spring(x_F, z_up_tF + z_mass/2,...
        x_F, z_up_sF - z_mass/2*cos(theta(i)),...
        nb_coil,h_susp,x_tire/5);

    z_up_sR  = h(i) - b*sin(theta(i));
    z_up_tR = z_up_sR - (h_susp - qsR(i));
    [x_spring_sR,y_spring_sR] = spring(x_R, z_up_tR + z_mass/2,...
        x_R, z_up_sR - z_mass/2*cos(theta(i)),...
        nb_coil,h_susp,x_tire/5);

    % Tire spring
    z_low_tF = z_up_tF - rwF;
    [x_spring_tF,y_spring_tF] = spring(x_F,z_low_tF,...
        x_F,z_up_tF - z_mass/2,...
        nb_coil,h_susp,x_tire/5);

    z_low_tR = z_up_tR - rwR;
    [x_spring_tR,y_spring_tR] = spring(x_R,z_low_tR,...
        x_R,z_up_tR - z_mass/2,...
        nb_coil,h_susp,x_tire/5);

%     % Plot point for debug
%     plot([x_F x_F x_F x(i) x_R x_R x_R ],...
%         [z_low_tF z_up_tF z_up_sF h(i) z_up_sR z_up_tR z_low_tR],'ko')

    % Update springs
    set(spring_sF,'XData',x_spring_sF,'YData',y_spring_sF)
    set(spring_sR,'XData',x_spring_sR,'YData',y_spring_sR)
    set(spring_tF,'XData',x_spring_tF,'YData',y_spring_tF)
    set(spring_tR,'XData',x_spring_tR,'YData',y_spring_tR)

    %% Compute position of rectangle for each mass (no rotation)
    % The position of each rectangle is:
    % [x coordinate of lower left edge; z coordiante of lower left edge;
    % x length; zlength]

    % Sprung mass (no rotation)
    set(sprung_mass,...
        'XData',x(i) + [-b-x_tire/2,...
        -b-x_tire/2,...
         a+x_tire/2,...
         a+x_tire/2,...
        -b-x_tire/2],...
        'YData',h(i) + [-z_mass/2,...
         z_mass/2,...
         z_mass/2,...
        -z_mass/2,...
        -z_mass/2])
    % Rotate chassis
    rotate(sprung_mass, [0 0 1], theta(i)*180/pi, [x(i) h(i) 0]) 

    % Front unsprung mass
    unsprungF_rect_pos = [x_F-x_tire/2,...
        z_up_tF - z_mass/2,...
        x_tire,...
        z_mass];

    % Rear unsprung mass
    unsprungR_rect_pos = [x_R-x_tire/2,...
        z_up_tR - z_mass/2,...
        x_tire,...
        z_mass];

    % Update masses
    set(unsprungF_mass,'pos',unsprungF_rect_pos);
    set(unsprungR_mass,'pos',unsprungR_rect_pos);

    %% Set axis and update frame
    axis equal
    ylim([h(i)-.7 h(i)+.3])
%     ylim([-0.02 0.06])
    xlim([x(i)-1.5*b x(i)+1.5*a])
    drawnow()
    frame(i) = getframe(gcf);
end

end





