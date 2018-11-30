function ineq = MPC_constraint(state,input,param)
%MPC_CONSTRAIN Define path constraint of the MPC

%% Parameters of semi-active suspension
b1 = 3000/0.4;
a1 = 0;
b2 = 1200/0.4;
a2 = 1800;
b3 = 1000/0.6;
a3 = 100;
b4 = 1000/0.6;
a4 = -100;
b5 = 1200/0.4;
a5 = -1800;

msp = param.msp;
mus = param.mus;

%% Define suspension velocity and actuator force
pspr = state(1);    % Sprung mass momentum
puns = state(2);    % Unsprung mass momentum
Fc   = input(1);

v = pspr/msp - puns/mus;

if v < (a1-a4)/(b4-b1)
    ineq(1) = -Fc + (b1*v+a1);
    ineq(2) =  Fc - (b4*v+a4);
    ineq(3) = -Fc + (b5*v+a5);
elseif v <= (a1-a3)/(b3-b1)
    ineq(1) =  Fc - (b1*v+a1);
    ineq(2) = -Fc + (b1*v+a1);
else
    ineq(1) =  Fc - (b1*v+a1);
    ineq(2) =  Fc - (b2*v+a2);
    ineq(3) = -Fc + (b3*v+a3);
end

end

