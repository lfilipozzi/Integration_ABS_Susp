mpc = import_mpctools();

% Define model.
Delta = 0.5;                    % sampling time
Nsim = floor(20*0.5/Delta);     % simulation steps
Nx = 2;                         % dimension of the states
Nu = 1;                         % dimension of the input
Nt = 20;                        % prediction horizon
r = 2;                          % constant position setpoint

% Build system model with Casadi Integrator
dint = mpc.getCasadiIntegrator(@ode, Delta, [Nx, Nu], {'x', 'u'},{'dint'});
kwargs = struct('funcname', 'ode');
Nc = 3;

% Define the control-oriented model in both linear and nonlinear settings
fnonlin = mpc.getCasadiFunc(@ode, [Nx, Nu], {'x', 'u'}, '**', kwargs);

% Define nonlinear model
linmodel = mpc.getLinearizedModel(fnonlin, ...
    {zeros(Nx, 1), zeros(Nu, 1)}, {'A', 'B'}, Delta); % Model linearization
Flin = mpc.getCasadiFunc(@(x, u) linmodel.A*x + linmodel.B*u, ...
    [Nx, Nu], {'x', 'u'}, {'dintlin'}); % Define linear model
% Define cost functions
l = mpc.getCasadiFunc(@(x,u) stagecost(x,u,r), [Nx, Nu], ...
    {'x','u'}, {'l'});
Vf = mpc.getCasadiFunc(@(x) termcost(x,r), Nx, {'x'}, {'Vf'});

% Specify bounds.
max_x1 = 2.1*Inf;
max_u = 0.1;
max_x2 = 0.25;
max_usim =0.1;
lb = struct();
lb.u = -ones(Nu, Nt)*max_u;
lb.x = -[ones(1,Nt+1)*Inf;ones(1,Nt+1)*max_x2];
ub = struct();
ub.u = ones(Nu, Nt)*max_u;
ub.x = [ones(1, Nt+1)*max_x1;ones(1,Nt+1)*max_x2];

% Build solvers.
N = struct('x', Nx, 'u', Nu, 't', Nt);
kwargs = struct('l', l, 'Vf', Vf, 'lb', lb, 'ub', ub, 'verbosity',3);
solvers = struct();
solvers.LMPC = mpc.nmpc('f', Flin, 'N', N, '**', kwargs);
N.c = Nc; % Collocation for nonlinear mpc.
solvers.NMPC = mpc.nmpc('f', fnonlin, 'N', N, 'Delta', Delta, '**', ...
    kwargs);

%% Simulate closed-loop.
data = struct(); controllers = fieldnames(solvers);
for i = 1:length(controllers)
    c = controllers{i}; solver = solvers.(c);
    fprintf('Simulating %s\n', c);
    x = NaN(Nx, Nsim + 1);
%     xc = NaN(Nx, Nc, Nsim);
    x(:,1) = [1; 0]; % Initial condition.
    u = NaN(Nu, Nsim);
    for k = 1:Nsim
        % Solve MPC problem.
        solver.fixvar('x', 1, x(:,k));
        solver.solve();
        if ~isequal(solver.status, 'Solve_Succeeded')
            warning('Solver failed at time %d!', k);
            break
        end
        u(:,k) = max(-max_usim,min(max_usim,solver.var.u(:,1)));
        x(:,k + 1) = full(dint(x(:,k), u(:,k))); % Simulate system.
    end
    % Store data.
    data.(c) = struct('x', x, 'u', u, 't', Delta*(0:Nsim));
end

% Make a phase plot.
colors = {'r', 'g'};
figure();
hold('on');
for i = 1:length(controllers)
    c = controllers{i};
    plot(data.(c).x(1,:), data.(c).x(2,:), ['-o', colors{i}]);
end
xlabel('Position');
ylabel('Velocity');
legend(controllers{:});
% Also make a timeseries plot for each.
for i = 1:length(controllers)
    c = controllers{i};
    mpc.mpcplot('x', data.(c).x, 'u', data.(c).u, 't', data.(c).t,...
        'title', c, 'xnames', {'Position', 'Velocity'}, ...
        'unames', {'Control'});
end

%%%****************************************************************
%%% Subfunctions
%%%****************************************************************
% System model
function dxdt = ode(x, u)
g_l = 0;
dxdt = [x(2); sqrt(g_l)*sin(x(1)) + u];
end
% Stage cost function
function l = stagecost(x, u,r)
l = (x(1)-r)^2+0.01*(x(2))^2+0.01*u^2;
end
% Terminal cost function
function Vf = termcost(x,r)
Vf = 1*(x(1)-r)^2+0.01*(x(2))^2;
end





