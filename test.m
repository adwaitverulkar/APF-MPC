% first casadi test for mpc fpr mobile robots
clear all
close all
clc

% CasADi v3.4.5
% addpath('C:\Users\mehre\OneDrive\Desktop\CasADi\casadi-windows-matlabR2016a-v3.4.5')
% CasADi v3.5.5
addpath('..\casadi-3.6.3-windows64-matlab2018b\')


import casadi.*


%% Create optimal control function using Opti class

mass = 4.72;   % mass of the RC car
I_z = 0.2036;  % Moment of Inertia
l_r = 0.131;   % length of the rear axle from centre
l_f = 0.189;   % length of front axle from centre
a_g = 9.81;    % acceleration due to gravity

T = 0.1;

F_max = 4; F_min = -F_max; % Thrust Bounds
d_delta_max = +0.03*pi; d_delta_min = -d_delta_max; % Steering Rate Bounds

x_init = [generate_initial_point();  pi/2; 0.6; 0.0; 0.0; 0.0]; % initial condition.
obs_loc = generate_obs_point();

x = SX.sym('x'); y = SX.sym('y'); psi = SX.sym('psi');
v_x = SX.sym('v_x'); v_y = SX.sym('v_y'); r = SX.sym('r');
delta = SX.sym('delta');

states = [x;y;psi;v_x;v_y;r;delta]; n_states = length(states);

F_x = SX.sym('F_x'); d_delta = SX.sym('d_delta');
controls = [F_x; d_delta]; n_controls = length(controls);


rhs = [states(4)*cos(states(3)) - states(5)*sin(states(3))
       states(4)*sin(states(3)) + states(5)*cos(states(3))
       states(6)
       controls(1)/mass
       (states(4)*controls(2) + states(7)*controls(1)/mass)*(l_r/(l_r+l_f))
       (states(4)*controls(2) + states(7)*controls(1)/mass)*(1/(l_r+l_f))
       controls(2)];

f = Function('f',{states, controls},{rhs}); % nonlinear mapping function f(x,u) for state dynamics


L_val = 1e5; % Goal potential param
k_val = 0.2; % Goal potential param

N = 20;

xs = [4.0; 1.0;  pi/2; 0.0; 0.0; 0.0; 0.0]; % Goal posture


scale = 0.5; % Scale of the obstacle
rx = [-3.3, -1.7
      1.7, 3.3
      -3.3, -1.7 
      1.7, 3.3];
 
ry =  [-3.3,-1.7
       -3.3, -1.7
       1.7, 3.3
       1.7, 3.3];

d_0_val = 0.2;

rects =[rx ry];
n_rects = size(rects,1);

nu_val = 1e5; % static obstacle potential param

% For circular dynamic obstacles: 
obs_diam = 0.4;
factor = SX.sym('factor',1);
factor_val = 1.00;
gaussian_amp = 1e5;
sigma_x = 0.3;
sigma_y = 0.3;
C = 0;


% Define the Weighting factor
Q = 1e-5* diag([2e5;2e5;2e5;2e5;2e5;1e-10;1e-10]); 
Q_N = 1e-4* diag([5e8;5e8;8e6;2e8;2e8;1e-10;1e-10]); % Hint : Q_N (high)
R = diag([1e1;1e1]);
S = 1e0*diag([1e1;1e3]);


opti = Opti();

% Define variables and parameters:
X = opti.variable(n_states, N+1);
U = opti.variable(n_controls, N);
P = opti.parameter(n_states+2);

st  = X(:,1); % initial state

obj = 0; % Objective function

g = [];  % constraints vector

g = [g; st-P(1:n_states)]; % initial condition constraints

for k = 1:N
    
    st = X(:,k);  con = U(:, k);
    U_rect_pot = full(repulsive_pot_lane(X(1,k), X(2,k), rx, ry, d_0_val, nu_val));

    U_goal_pot = full(U_attractive_pot(X(1,k),X(2,k),xs(1),xs(2),L_val,k_val)) - ...
                 invertedGaussian(X(1,k),X(2,k), gaussian_amp, xs(1), xs(2), sigma_x, sigma_y, C) + ...
                 invertedGaussian(X(1,k),X(2,k), gaussian_amp, P(end-1), P(end), sigma_x, sigma_y, C);
    U_tot = U_rect_pot + U_goal_pot;

    if k < N
        con_next = U(:,k+1);
    end

    if k ~=N
        U_obs = full(invertedGaussian(X(1,k), X(2,k), P(end-1), P(end), d_0_val, nu_val,factor_val^k));
        obj = obj+(st-P(1:n_states))'*Q*(st-P(1:n_states)) ...
        + con'*R*con + (con_next-con)'*S*(con_next-con) + U_tot + U_obs; % calculate obj   
    else
        U_obs = full(invertedGaussian(X(1,k),X(2,k), P(end-1), P(end),d_0_val,nu_val,factor_val^k));
        obj = obj+(st-P(1:n_states))'*Q_N*(st-P(1:n_states)) ...
                + con'*R*con + (con_next-con)'*S*(con_next-con) + U_tot + U_obs; % calculate obj
    end
    st_next = X(:, k+1);

    % Runge - Kutta 4th order:
    k1 = f(st, con);
    k2 = f(st + 0.5*T*k1, con);
    k3 = f(st + 0.5*T*k2, con);
    k4 = f(st + T*k3, con);
    st_next_RK4 = st + T/6 *(k1 + 2*k2 + 2*k3 +k4);



    g = [g;  st_next-st_next_RK4]; % compute constraints
end

opti.minimize(obj)
opti.subject_to(g==0)

opti.subject_to(X(1, :) >= -5)
opti.subject_to(X(1, :) <= 10)

opti.subject_to(X(2, :) >= -5)
opti.subject_to(X(2, :) <= 10)

opti.subject_to(X(3, :) >= -pi)
opti.subject_to(X(3, :) <= pi)

opti.subject_to(X(4, :).' >= -0.5)
opti.subject_to(X(4, :) <= 2)

opti.subject_to(X(5, :) >= -0.5)
opti.subject_to(X(5, :) <= 0.5)

opti.subject_to(X(6, :) >= -2*pi)
opti.subject_to(X(6, :) <= 2*pi)

opti.subject_to(X(7, :) >= -2*pi)
opti.subject_to(X(7, :) <= 2*pi)

opti.subject_to(U(1, :) >= F_min)
opti.subject_to(U(1, :) <= F_max)

opti.subject_to(U(2, :) >= d_delta_min)
opti.subject_to(U(2, :) <= d_delta_max)

opti.solver('ipopt')

opt_cont = opti.to_function('opt_cont', {P}, {[X(:, 2:N+1); U]});
sol = opt_cont([x_init; obs_loc(1); obs_loc(2)]);




