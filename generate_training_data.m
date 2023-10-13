% first casadi test for mpc fpr mobile robots
clear all
close all
clc

% CasADi v3.4.5
% addpath('C:\Users\mehre\OneDrive\Desktop\CasADi\casadi-windows-matlabR2016a-v3.4.5')
% CasADi v3.5.5
addpath('..\casadi-3.6.3-windows64-matlab2018b\')
% addpath('D:\Matlab\casadi-3.6.3-windows64-matlab2018b')
% addpath('/home/anshulnayak/MPC/Matlab/Casadi/casadi-3.6.3-linux64-matlab2018b')

import casadi.*

%% Simulation Parameters 

T = 0.1; % MPC time step (s)
N = 20; % prediction horizon
rob_diam = 0.6; % Robot Diameter (m)

F_max = 4; F_min = -F_max; % Thrust Bounds
d_delta_max = +0.03*pi; d_delta_min = -d_delta_max; % Steering Rate Bounds


%% Define Symbolic Variables

x = SX.sym('x'); y = SX.sym('y'); psi = SX.sym('psi');
v_x = SX.sym('v_x'); v_y = SX.sym('v_y'); r = SX.sym('r');
delta = SX.sym('delta');

states = [x;y;psi;v_x;v_y;r;delta]; n_states = length(states);

F_x = SX.sym('F_x'); d_delta = SX.sym('d_delta');
controls = [F_x; d_delta]; n_controls = length(controls);


%% Robot Parameters

mass = 4.72;   % mass of the RC car
I_z = 0.2036;  % Moment of Inertia
l_r = 0.131;   % length of the rear axle from centre
l_f = 0.189;   % length of front axle from centre
a_g = 9.81;    % acceleration due to gravity

xs = [4.0; 1.0;  pi/2; 0.0; 0.0; 0.0; 0.0]; % Goal posture

L_val = 1e5; % Goal potential param
k_val = 0.2; % Goal potential param

%% State Dynamics
rhs = [states(4)*cos(states(3)) - states(5)*sin(states(3))
       states(4)*sin(states(3)) + states(5)*cos(states(3))
       states(6)
       controls(1)/mass
       (states(4)*controls(2) + states(7)*controls(1)/mass)*(l_r/(l_r+l_f))
       (states(4)*controls(2) + states(7)*controls(1)/mass)*(1/(l_r+l_f))
       controls(2)];

f = Function('f',{states, controls},{rhs}); % nonlinear mapping function f(x,u) for state dynamics
%% Define Optimization Horizons

n_obs = 1;

% U = SX.sym('U',n_controls,N); % Decision variables (controls)
% P = SX.sym('P', n_states + n_states + (N+1)*n_obs); % parameters (which include at the initial state of the robot and the reference state)
% 
% X = SX.sym('X', n_states, (N+1)); % A vector that represents the states over the optimization problem.

%% Static Obstacle Locations

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


%% Changing Obstacle Parameters

% For circular dynamic obstacles: 
obs_diam = 0.4;
factor = SX.sym('factor',1);
factor_val = 1.00;
gaussian_amp = 1e5;
sigma_x = 0.3;
sigma_y = 0.3;
C = 0;

%% MPC tuning parameters
% Define the Weighting factor
Q = 1e-5* diag([2e5;2e5;2e5;2e5;2e5;1e-10;1e-10]); 
Q_N = 1e-4* diag([5e8;5e8;8e6;2e8;2e8;1e-10;1e-10]); % Hint : Q_N (high)
R = diag([1e1;1e1]);
S = 1e0*diag([1e1;1e3]);

%% Generate MPC Objective Function
opti = Opti();

% Define variables and parameters:
X = opti.variable(n_states, N+1);
U = opti.variable(n_controls, N);
P = opti.parameter(n_states+2*n_obs);

st  = X(:,1); % initial state

obj = 0; % Objective function

g = [];  % constraints vector
g = [g; st-P(1:n_states)]; % initial condition constraints

for k = 1:N  
    st = X(:,k);  con = U(:, k);
    U_rect_pot = full(repulsive_pot_lane(X(1, k), X(2, k), rx, ry, d_0_val, nu_val));

    U_goal_pot = full(U_attractive_pot(X(1,k),X(2,k),xs(1),xs(2),L_val,k_val)) - ...
                 invertedGaussian(X(1,k),X(2,k), gaussian_amp, xs(1), xs(2), sigma_x, sigma_y, C) + ...
                 invertedGaussian(X(1,k),X(2,k), gaussian_amp, P(end-1), P(end), sigma_x, sigma_y, C);
    
    U_tot = U_rect_pot + U_goal_pot;

    if k < N
        con_next = U(:,k+1);
    end

    if k ~=N
        % U_obs = full(invertedGaussian(X(1,k), X(2,k), P(end-1), P(end), d_0_val, nu_val,factor_val^k));
        obj = obj+(st-xs)'*Q*(st-xs) ...
        + con'*R*con + (con_next-con)'*S*(con_next-con) + U_tot; % calculate obj   
    else
        % U_obs = full(invertedGaussian(X(1,k),X(2,k), P(end-1), P(end),d_0_val,nu_val,factor_val^k));
        obj = obj+(st-xs)'*Q_N*(st-xs) ...
                + con'*R*con + (con_next-con)'*S*(con_next-con) + U_tot; % calculate obj
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

opti.subject_to(X(:, 1) == P(1:n_states))

opti.subject_to(-5 <= X(1, :) <= 10)
opti.subject_to(-5 <= X(2, :) <= 10)
opti.subject_to(-pi <= X(3, :) <= pi)
opti.subject_to(-0.5 <= X(4, :) <= 2)
opti.subject_to(-0.5 <= X(5, :) <= 0.5)
opti.subject_to(-2*pi <= X(6, :) <= 2*pi)
opti.subject_to(-2*pi <= X(7, :) <= 2*pi)

opti.subject_to(F_min <= U(1, :) <= F_max)
opti.subject_to(d_delta_min <= U(2, :) <= d_delta_max)

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0; %0,3
opts.print_time = 0;
% opts.ipopt.linear_solver = 'mumps';
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

opti.solver('ipopt', opts)

opt_cont = opti.to_function('opt_cont', {P}, {[X(:, 2:N+1); U]});


%% Image generation loop

MPC_runs = 1;
for current_run = 1:MPC_runs
    
    % Define the initial and Goal States:
    x0 = [generate_initial_point();  pi/2; 0.6; 0.0; 0.0; 0.0]; % initial condition.
    x0 = [1.0; -3.0;  pi/2; 0.6; 0.0; 0.0; 0.0]; 
    obs_loc = generate_obs_point();
    obs_loc = [2; -1];
    obs_loc_x = obs_loc(1);
    obs_loc_y = obs_loc(2);

    goal_pos = [xs(1); xs(2)];

    t0 = 0;

    xx(:, 1) = x0; % xx contains the history of states
    t(1) = t0;
    
    u0 = zeros(N,2);        % two control inputs for each robot
    X0 = repmat(x0,1,N+1)'; % initialization of the states decision variables
    
    sim_tim = 10; % Maximum simulation time
    
    % Start MPC:
    mpciter = 0;
    xx1 = [];
    u_cl = [];
    
    % the main simulaton loop... it works as long as the error is greater
    % than 10^-6 and the number of mpc steps is less than its maximum
    % value.
    main_loop = tic;
    tol = 3e-1;
    obs_cl = [];

    while(norm((x0(1:2)-xs(1:2)), 2) > tol && mpciter < sim_tim / T)

        tic
        % generate_APF_image(x0(1:2), goal_pos, obs_loc, sigma_x, sigma_y, gaussian_amp, L_val, k_val, rx, ry, d_0_val, nu_val)

        % image_filename = sprintf('./images/image_%d_%d.png', current_run, mpciter);
        % Save the figure without the gray area using exportgraphics
        % exportgraphics(gcf, image_filename, 'Resolution', 300, 'ContentType', 'auto');


        % args.p(1:2*n_states) = [x0; xs]; % set the values of the parameters vector
        
        % Update the obstacle location:
        for k = 1:N+1
            t_predicted = (k-1)*T;
            obs_vel_x = 0.0;
            obs_vel_y = 0.4;
            obs_x = obs_loc_x + 0*t_predicted*obs_vel_x;
            obs_y = obs_loc_y + 0*t_predicted*obs_vel_y;

            args.p(n_states*2+n_obs*k-1) = obs_x;
            args.p(n_states*2+n_obs*k)   = obs_y;  

            obs_cl(k,1:2,mpciter+1) = [obs_x,obs_y];

        end
        % obs_loc_x = obs_loc_x + T * obs_vel_x;
        % obs_loc_y = obs_loc_y + T * obs_vel_y;
    
        % initial value of the optimization variables
        args.x0  = [reshape(X0',n_states*(N+1),1);reshape(u0',n_controls*N,1)];

        sol = opt_cont([x0; obs_loc]);
        u = full(sol(end-1:end, :).');

        xx1(:,1:7,mpciter+1)= [x0, full(sol(1:n_states,:))].';

        u_cl= [u_cl ; u(1,:)];

        t(mpciter+1) = t0;
    
        % Apply the control and shift the solution
        [t0, x0, u0] = shift(T, t0, x0, u,f);

        xx(:, mpciter+2) = x0;
        X0 = [x0, full(sol(1:n_states,:))]; % get solution TRAJECTORY
        % Shift trajectory to initialize the next step
        X0 = [X0(2:end,:); X0(end,:)];
        ss_error = norm((x0(1:2)-xs(1:2)),2)
        mpciter = mpciter + 1;
    end
    data_filename = sprintf('./data/data_%d.mat', current_run);
    save(data_filename, 'xx', 'u_cl');
    toc

end














