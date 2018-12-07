clear; clc;
close all;

%% Create system: double integrator
% The system is given by:
%       x_dot = A_c x + B_c u + Bd_c w
%       y     = C_c x + D_c u + Dd_c w
% with x the states, u the control input, w the plant input disturbance and
% y the output.

A_c  = [0 1;0 0];
B_c  = [0; 1];
Bd_c = [0; 1];
C_c  = [1 0];
D_c  = 0;
Dd_c = 0;

sys_c = ss(A_c,[B_c Bd_c],C_c,[D_c Dd_c]);
    
Nx = size(A_c,1);   % Number of states
Nu = size(B_c,2);   % Number of control inputs
Nw = size(Bd_c,2);  % Number of input disturbances
Ny = size(C_c,1);   % Number of outputs

% Discretization
Ts = 0.1;
sys = c2d(sys_c, Ts);
A  = sys.A;
B  = sys.B(:,1:Nu);
Bd = sys.B(:,1+Nu:end);
C  = sys.C;
D  = sys.D(:,1:Nu);
Dd = sys.D(:,1+Nu:end);

%% MPC quadratic cost function
% The cost function is:
%       J = x_N' P x_N + sum(x_j' Q x_j + u_j' R u_j)
Q = rand(Nx); Q = Q'*Q;    % Weight on state x
R = rand(Nu); R = R'*R;    % Weight on input u
P = dare(A,B,Q,R);  % Terminal weight
Nt = 5;        % Prediction horizon

% The control law is given by:
%       u_j = K x_j + v_j
K = dlqr(A,B,Q,R);
AK = A + B*K;

% Delete A to be sure it is not used latter
clear A

%% MPC linear inequalities constraints
% Constraints: Constraints are written as:
%       F x + G v <= m + M w

% Only consider bounds on u for now
%       umin <= u_k <= umax, k = 0, ..., N-1
umax = rand(Nu,1);
umin = -umax;

% Bounds on disturbance
%       wmin <= w_k <= wmax, k = 0, ..., N-1
wmax = rand(Nw,1);
wmin = -wmax;

% Check size:
if not(prod(size(umax) == [Nu, 1]))
    error('Incorect size for uub')
end
if not(prod(size(umin) == [Nu, 1]))
    error('Incorect size for ulb')
end
if not(prod(size(wmax) == [Nw, 1]))
    error('Incorect size for wub')
end
if not(prod(size(wmin) == [Nw, 1]))
    error('Incorect size for wlb')
end

% Check min <= max
if sum(umin > umax)
    error('Unfeasible bounds on u')
end
if sum(wmin > wmax)
    error('Unfeasible bounds on w')
end

% Extend matrices umin, umax to obtain uub and ulb where:
%       ulb <= u <= uub
% with:
%       u = [u0 ... uN-1]^T
uub = zeros(Nu*Nt,1);
for k = 1:Nt
    uub((k-1)*Nu+1:k*Nu,:) = umax;
end

ulb = zeros(Nu*Nt,1);
for k = 1:Nt
    ulb((k-1)*Nu+1:k*Nu,:) = umin;
end

%% Batch approach
% Write the state equation over the prediction horizon:
%       x = Sxx x0 + Sxv v + Sxw w
% where:
%       x = [x0 ... xN]^T
%       v = [v0 ... vN-1]^T
%       w = [w0 ... wN-1]^T

Sxx = zeros(Nx*(Nt+1),Nx);
Sxx(1:Nx,:) = eye(Nx);
for i = 1:Nt
    Sxx(i*Nx+1:(i+1)*Nx,:) = AK * Sxx((i-1)*Nx+1:i*Nx,:);
end

Sxv = zeros(Nx*(Nt+1),Nu*Nt);
% Sv(Nx*Nt+1:end,Nu*(Nt-1)+1:end) = B;
for i = flip(2:Nt+1)
    Sxv(Nx*(i-1)+1:Nx*i, Nu*(i-2)+1:Nu*(i-1)) = B;
    for j = flip(1:i-2)
        Sxv((i-1)*Nx+1:i*Nx, (j-1)*Nu+1:j*Nu) = ...
            AK * Sxv((i-1)*Nx+1:i*Nx, j*Nu+1:(j+1)*Nu);
    end
end

Sxw = zeros(Nx*(Nt+1),Nw*Nt);
% Sv(Nx*Nt+1:end,Nu*(Nt-1)+1:end) = B;
for i = flip(2:Nt+1)
    Sxw(Nx*(i-1)+1:Nx*i, Nw*(i-2)+1:Nw*(i-1)) = Bd;
    for j = flip(1:i-2)
        Sxw((i-1)*Nx+1:i*Nx, (j-1)*Nw+1:j*Nw) = ...
            AK * Sxw((i-1)*Nx+1:i*Nx, j*Nw+1:(j+1)*Nw);
    end
end

% Write the control input equation over the prediction horizon
%       u = Sux x0 + Suv v + Suw w
% where:
%       u = [u0 ... uN-1]^T
%       v = [v0 ... vN-1]^T
%       w = [w0 ... wN-1]^T

KK = zeros(Nt*Nu,Nt*Nx);
for k = 1:Nt
    KK((k-1)*Nu+1:k*Nu,(k-1)*Nx+1:k*Nx) = K;
end

Sux = KK * Sxx(1:Nt*Nx,:);

Suv = eye(Nt*Nu);
Suv(Nu+1:end,1:Nu) = Sux(1:(Nt-1)*Nu,:) * B;
for j = 1:Nt-1
    Suv(j*Nu+1:end,(j-1)*Nu+1:j*Nu) = Suv(Nu+1:(Nt-j+1)*Nu,1:Nu);
end

% Possible error here:
% Suw = zeros(Nt*Nu);
% Suw(Nu+1:end,1:Nu) = Sux(1:(Nt-1)*Nu,:) * Bd;
% for j = 1:Nt-1
%     Suw(j*Nu+1:end,(j-1)*Nu+1:j*Nu) = Suw(Nu+1:(Nt-j+1)*Nu,1:Nu);
% end
% Use the floowing code instead
Suw = zeros(Nt*Nu,Nt*Nw);
Suw(Nu+1:end,1:Nw) = Sux(1:(Nt-1)*Nu,:) * Bd;
for j = 1:Nt-1
    Suw(j*Nu+1:end,(j-1)*Nw+1:j*Nw) = Suw(Nu+1:(Nt-j+1)*Nu,1:Nw);
end

% Write cost function over the prediction horizon:
%       J = x' QQ x + u' RR u
QQ = zeros(Nx*(Nt+1));
for k = 1:Nt
    QQ((k-1)*Nx+1:k*Nx,(k-1)*Nx+1:k*Nx) = Q;
end
QQ(Nt*Nx+1:(Nt+1)*Nx,Nt*Nx+1:(Nt+1)*Nx) = P;

RR = zeros(Nu*Nt);
for k = 1:Nt
    RR((k-1)*Nu+1:k*Nu,(k-1)*Nu+1:k*Nu) = R;
end

% Delete Q, R, and P to be sure it is not used later
clear P Q R

% Reformulate the cost function using matrices Hx, Hv, and Hw:
%       J = || Hx x + Hv v + Hw w ||_2^2
% From the matrices that have been computed, the cost can be written as:
%   J = [x0' v' w'] * HH * [x0; v; w]
% with HH a symmetric positive semi-definit matrix
% 
% HH = [(Sxx'*QQ*Sxx + Sux'*RR*Sux) (Sxx'*QQ*Sxv + Sux'*RR*Suv) (Sxx'*QQ*Sxw + Sux'*RR*Suw);
%       (Sxv'*QQ*Sxx + Suv'*RR*Sux) (Sxv'*QQ*Sxv + Suv'*RR*Suv) (Sxv'*QQ*Sxw + Suv'*RR*Suw);
%       (Sxw'*QQ*Sxx + Suw'*RR*Sux) (Sxw'*QQ*Sxv + Suw'*RR*Suv) (Sxw'*QQ*Sxw + Suw'*RR*Suw)];
% 
% Where:
%   HH = [ Hx'*Hx   Hx'*Hv   Hx'*Hw;
%          Hv'*Hx   Hv'*Hv   Hv'*Hw;
%          Hw'*Hx   Hw'*Hv   Hw'*Hw];

HxtHx = Sxx'*QQ*Sxx + Sux'*RR*Sux;
HxtHv = Sxx'*QQ*Sxv + Sux'*RR*Suv;
HxtHw = Sxx'*QQ*Sxw + Sux'*RR*Suw;
HvtHv = Sxv'*QQ*Sxv + Suv'*RR*Suv;
HvtHw = Sxv'*QQ*Sxw + Suv'*RR*Suw;
HwtHw = Sxw'*QQ*Sxw + Suw'*RR*Suw;
HvtHx = Sxv'*QQ*Sxx + Suv'*RR*Sux;
HwtHx = Sxw'*QQ*Sxx + Suw'*RR*Sux;
HwtHv = Sxw'*QQ*Sxv + Suw'*RR*Suv;

%% Compute all vertices of the disturbance set
% Create all possible combination of disturbances given the bounds on w
% over the prediction horizon. Possible combinations are:
%       w_1 = [wmin ... wmin wmin wmin]^T
%       w_2 = [wmin ... wmin wmin wmax]^T
%       w_3 = [wmin ... wmin wmax wmin]^T
%       w_4 = [wmin ... wmin wmax wmax]^T
%       ...
%       w_{2^(Nw*Nt)} = [wmax ... wmax]^T
% The number of combination is exponential with the prediction horizon and
% number of disturbances.
% 
% To do that we create w_min_max the vector of extreme values for w for one
% timestep. Then, w_min_max is a cell structure of extreme values for each
% timestep over the prediction horizon. Finally, w_vertices is the vector
% of all possible combinations.

w_min_max = [wmin wmax];
w_min_max_horizon = cell(Nt,1);
for k = 1:Nt
    w_min_max_horizon{k} = w_min_max;
end
w_vertices = allcomb(w_min_max_horizon{:})';

% Each row of w_vertices correspond to a possible combination of
% disturbance at one timestep of the prediction horizon.

% TODO: this only work for one disturbance for now (Nw = 1)

%% Transform constraint on x0 and u into constraints on x0 and v
% With the following bounds on u:
%       ulb <= u <= uub
% we have:
%       ulb <= Sux x0 + Suv v + Suw w <= uub
% or equivalently
%      -Sux x0 - Suv v <= - ulb + Suw w
%       Sux x0 + Suv v <=   uub - Suw w

% Constraints can now be written as:
%       F x0 + Gv <= m + M w
F = [-Sux; Sux];
G = [-Suv; Suv];
m = [-ulb; uub];
M = [Suw; -Suw];

% Constraints can be written as:
%       F x0 + Gv <= d
% with:
%       d = m + max_{w} M*w
d = m + max(M*w_vertices,[],2);

% TODO: for now only bounds on u are considered, need to add bounds on x
% and polytopic constraints ax x +au u <= b

%% Transform min-max optimization problem into mp-QP
% The min-max MPC problem is equivalent to the following QP problem:
%   min_{v, gamma} || Hx * x + Hv * v + Hw * 0 ||_2^2 + gamma
%   subject to:   F*v + G*v <= d
%                 gamma >= gama_min(w) for all w
% 
% with gamma_min(w) = || Hx * x + Hv * v + Hw * w ||_2^2 
%                   - || Hx * x + Hv * v + Hw * 0 ||_2^2
%                   = w' Hw'Hw w + 2*w'Hw'*(Hx x + Hv v)
%
% To obtain an equivalent problem in which the functional does not depends 
% on the state vector, we change variable
%       z = v + (Hv'*Hv)^-1 Hv'Hx x
% and the min max problem becomes
%   J = x0' Y x0 + min_{z, gamma} 1/2*z'*H*z + gamma
%   subject to: Gm z + gm*gamma <= Wm + Sm * x
%               Gc z <= Wc + Sc x
% with H = 2*Hv'*Hv

H = 2*HvtHv;

% After changing variable, the constraints become:
%   G z <= d + [G (Hv'Hv)^(-1) Hv'Hx - F] x
% or equavlaently:
%   Gc z <= Wc + Sc x
% and for all w vertex of W:
% 2 w' Hw'Hv z - gamma <= -w' Hw'Hw w + w' 2*[Hw'Hx - Hw'Hv (Hv'Hv)^(-1) Hv'Hx] x
% which can be written as 2^(Nw*Nt) inequalities as:
%   Gm z - gamma <= Wm + Sm x

Gc = G;
Wc = d;
Sc = G/HvtHv*HvtHx - F;

% Check Wm: size error
Gm = 2 * w_vertices' * HwtHv;
gm = -ones(2^(Nw*Nt),1);
Wm = zeros(2^(Nw*Nt),1);
for k = 1:2^(Nw*Nt)
    Wm(k) = -w_vertices(:,k)' * HwtHw * w_vertices(:,k);
end
Sm = 2 * w_vertices' * (HwtHx - HwtHv / HvtHv * HvtHx);

%% Reformulate the problem as a quadratic problem and solve it
% Initial state
x0 = rand(Nx,1);

% Make sure H is symmetric due to computation error
H = (H + H')/2;

H_QP = [H zeros(Nu*Nt,1); zeros(1,Nu*Nt) 0];
f_QP = [zeros(Nu*Nt,1); 1];
A_QP = [Gm gm; Gc zeros(2*Nt*Nu,1)];    % TODO: 2*Nt*Nu since only bounds on u are considered
b_QP = [(Wm + Sm*x0); (Wc + Sc*x0)];

z_QP = quadprog(H_QP,f_QP,A_QP,b_QP);
z  = z_QP(1:Nu*Nt);             % Get rid of gamma
v  = z - HvtHv \ HvtHx * x0;    % Compute sequence v
v0 = v(1:Nu);   % Select the first entry
u0 = K*x0 + v0; % Compute u0






















