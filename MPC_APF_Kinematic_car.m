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

tic;
T = 0.1; %[
N = 20; % prediction horizon
rob_diam = 0.6;

F_max = 4; F_min = -F_max;
d_delta_max = +0.03*pi; d_delta_min = -d_delta_max;

x = SX.sym('x'); y = SX.sym('y'); psi = SX.sym('psi');
v_x = SX.sym('v_x'); v_y = SX.sym('v_y'); r = SX.sym('r');
delta = SX.sym('delta');


states = [x;y;psi;v_x;v_y;r;delta]; n_states = length(states);

F_x = SX.sym('F_x'); d_delta = SX.sym('d_delta');
controls = [F_x; d_delta]; n_controls = length(controls);

% Define the constants:
mass = 4.72;   % mass of the RC car
I_z = 0.2036;  % Moment of Inertia
l_r = 0.131;   % length of the rear axle from centre
l_f = 0.189;   % length of front axle from centre
a_g = 9.81;    % acceleration due to gravity

rhs = [states(4)*cos(states(3)) - states(5)*sin(states(3))
       states(4)*sin(states(3)) + states(5)*cos(states(3))
       states(6)
       controls(1)/mass
       (states(4)*controls(2) + states(7)*controls(1)/mass)*(l_r/(l_r+l_f))
       (states(4)*controls(2) + states(7)*controls(1)/mass)*(1/(l_r+l_f))
       controls(2)
        ]; % system r.h.s



% Define the obstacles:
obs_x = SX.sym('obs_x');
obs_y = SX.sym('obs_y');

obsts = [obs_x;obs_y];
n_obs = length(obsts);



f = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)f = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)
U = SX.sym('U',n_controls,N); % Decision variables (controls)
P = SX.sym('P', n_states + n_states + (N+1)*n_obs);
% parameters (which include at the initial state of the robot and the reference state)

X = SX.sym('X', n_states, (N+1));
% A vector that represents the states over the optimization problem.

obj = 0; % Objective function
g = [];  % constraints vector

% Define the initial and Goal States:
% x0 = [1.0; -3.0;  pi/2; 0.6; 0.0; 0.0; 0.0];    % initial condition.
x0 = [-0.1894; -3.7903;  pi/2; 0.6; 0.0; 0.0; 0.0];    % initial condition.
xs = [4.0; 1.0;  pi/2; 0.0; 0.0; 0.0; 0.0]; % Reference posture.


% Define Artificial Potential field (APF) for Lane boundaries of the road
% as a penalty to the cost function :

scale = 0.5; % Scale of the obstacle
rx = [-3.3, -1.7
         1.7, 3.3
        -3.3, -1.7 
        1.7, 3.3];
 
 ry =  [-3.3,-1.7
          -3.3, -1.7
          1.7, 3.3
          1.7, 3.3];
      
 d_0 = SX.sym('d_0');
 d_0_val = 0.2;
 
 
 rects =[rx ry];
 n_rects = size(rects,1);
 
%  % Define the APF cost function for lane constraints:
% g_APF = SX.zeros(1,1);
% 
% RX = SX.sym('RX',[n_rects,2]);
% RY = SX.sym('RY',[n_rects,2]);
% 
nu = SX.sym('nu');
nu_val = 1e5;
% 
% for i =1:n_rects
% %      rx1 =  RX(i,1);
% %      rx2 =  RX(i,2);
% %      ry1 =  RY(i,1);
% %      ry2 =  RY(i,2);
% 
%      g_APF = g_APF + nu * sum(exp( -((sqrt((max([RX(i,1)- x, 0,  x-RX(i,2)])).^2 + ...
%          (max([ RY(i,1)- y,  0,  y-RY(i,2)])).^2) + d_0)^2/(2*d_0^2))));
% end
% 
% U_rect = Function('U_rect', {x,y,RX,RY,d_0,nu}, {g_APF});


% For circular dynamic obstacles: 
% file_name = "Reactive_collision";
obs_diam = 0.4;
factor = SX.sym('factor',1);
factor_val = 1.00;
% obs_x = SX.sym('obs_x', 1);
% obs_y = SX.sym('obs_y', 1);
% n_circs = size(obs_x,1);
% 
% obs_loc_x = 2.0;
% obs_loc_y = -1.0;

obs_loc_x = -0.5509;
obs_loc_y = 0.5689;

obs_loc = [obs_loc_x; obs_loc_y];

% CX = SX.sym('CX',[n_circs,1]);
% CY = SX.sym('CY',[n_circs,1]);

% for i = 1:n_circs
%         d = max([sqrt((x-obs_x).^2 +(y-obs_y).^2) - obs_diam*factor,0]);
%         g_APF =  nu* sum(exp( -((d + d_0)^2/(2*d_0^2))));
% end
% 
%  U_circ = Function('U_circ', {x,y,obs_x,obs_y,d_0,nu,factor}, {g_APF});
% 
 
% Define attractive APF for Goal location:
g_goal = SX.zeros(1,1);
L = SX.sym('L');
L_val = 1e5;

k = SX.sym('k');
k_val = 0.2;

X_goal = SX.sym('X_goal');
Y_goal = SX.sym('Y_goal');

% Parameters
gaussian_amp = 1e5;
sigma_x = 0.3;
sigma_y = 0.3;
C = 0;

g_goal =  L/(1+exp(-k* sqrt((x-X_goal).^2 + (y-Y_goal).^2)));
U_goal = Function('U_goal',{x,y,X_goal,Y_goal,L,k},{g_goal});



% for i = 1:nx
%     parfor j = 1:ny
%          potential_field(i,j) =  full(U_rect(A(i,j),B(i,j), rx,ry,d_0_val,nu_val)) + ...
%                                     full(U_goal(A(i,j),B(i,j),xs(1),xs(2),L_val,k_val)) - ...
%                                     invertedGaussian(A(i,j),B(i,j), gaussian_amp, xs(1), xs(2), sigma_x, sigma_y, C) + ...
%                                     invertedGaussian(A(i,j),B(i,j), 5*gaussian_amp, obs_loc(1), obs_loc(2), sigma_x, sigma_y, C);
%     end
% end
init_point = generate_initial_point();
goal_pos = [xs(1); xs(2)];
obs_loc = generate_obs_point();
generate_APF_image(init_point, goal_pos, obs_loc, sigma_x, sigma_y, gaussian_amp, L_val, k_val, rx, ry, d_0_val, nu_val)
% 
% saveas(gcf, 'contour_plot.png');

% Save the figure without the gray area using exportgraphics
exportgraphics(gcf, 'contour_plot.png', 'Resolution', 300, 'ContentType', 'auto');


img_time = toc;
fprintf("Time consumed in map generation: %f secs \n", img_time)

fprintf("done")
%% non 
    
% Define the Weighting factor
Q = 1e-5* diag([2e5;2e5;2e5;2e5;2e5;1e-10;1e-10]); 
Q_N = 1e-4* diag([5e8;5e8;8e6;2e8;2e8;1e-10;1e-10]); % Hint : Q_N (high)
R = diag([1e1;1e1]);
S = 1e0*diag([1e1;1e3]);

% Q = zeros(7,7); Q(1,1) = 1;Q(2,2) = 5;Q(3,3) = 0.1; % weighing matrices (states)
% R = zeros(2,2); R(1,1) = 0.5; R(2,2) = 0.05; % weighing matrices (controls)

st  = X(:,1); % initial state
g = [g; st-P(1:n_states)]; % initial condition constraints
for k = 1:N
    st = X(:,k);  con = U(:,k);
    U_rect_pot = full(repulsive_pot_lane(X(1,k),X(2,k), rx,ry,d_0_val,nu_val));
    % U_circ_pot = full(U_circ(X(1,k),X(2,k), cx,cy,d_0_val,nu_val));
    U_goal_pot = full(U_attractive_pot(X(1,k),X(2,k),xs(1),xs(2),L_val,k_val)) - ...
                 invertedGaussian(X(1,k),X(2,k), gaussian_amp, xs(1), xs(2), sigma_x, sigma_y, C) + ...
                 invertedGaussian(X(1,k),X(2,k), gaussian_amp, obs_loc(1), obs_loc(2), sigma_x, sigma_y, C);
    U_tot = U_rect_pot + U_goal_pot;

    if k < N
        con_next = U(:,k+1);
    end
%     U_pot = Potential_field(st);
    if k ~=N
        U_obs = full(invertedGaussian(X(1,k),X(2,k), P(2*n_states+n_obs*k-1),P(2*n_states+n_obs*k),d_0_val,nu_val,factor_val^k));
        obj = obj+(st-P(n_states+1:2*n_states))'*Q*(st-P(n_states+1:2*n_states)) ...
        + con'*R*con + (con_next-con)'*S*(con_next-con) + U_tot + U_obs; % calculate obj   
    else
        U_obs = full(invertedGaussian(X(1,k),X(2,k), P(2*n_states+n_obs*k-1),P(2*n_states+n_obs*k),d_0_val,nu_val,factor_val^k));
        obj = obj+(st-P(n_states+1:2*n_states))'*Q_N*(st-P(n_states+1:2*n_states))...
        + con'*R*con + (con_next-con)'*S*(con_next-con) + U_tot + U_obs; % calculate obj
    end
    st_next = X(:, k+1);

    % Runge - Kutta 4th order:
    k1 = f(st, con);
    k2 = f(st + 0.5*T*k1, con);
    k3 = f(st + 0.5*T*k2, con);
    k4 = f(st + T*k3, con);
    st_next_RK4 = st + T/6 *(k1 + 2*k2 + 2*k3 +k4);

%     % Next State : Euler Integgration 
%     f_value = f(st,con);
%     st_next_euler = st + (T*f_value);

    g = [g;  st_next-st_next_RK4]; % compute constraints
end



% % Add constraints for collision avoidance
% obs_x = [2.5;  -1.6; -2] ; % meters
% obs_y = [-1; -0.8; -1]; % meters
% n_obstacles = length(obs_x);
% obs_diam = 0.4; % meters
% 
% 
% 
% for k = 1:N+1   % box constraints due to the map margins
%     for n = 1:n_obstacles
%         g = [g ; -sqrt((X(1,k)-obs_x(n))^2+(X(2,k)-obs_y(n))^2) + (rob_diam/2 + obs_diam/2); 
%             ];
%     end
% end


% Method 2: Define Lane boundaries as constraints
% % Define the coordinates of the line segment
% P_lane = [3 0;
%      3 3;
%      6 3
%      6 6
%      3 6];
% 
% 
% coord = Lane_obstacle(P_lane);
% N_seg = size(coord,1);
% 
% dia_lane_circ = 0.5;
% 
% 
% % 
% % p3 = [3 6];
% % p4 = [6 6];
% % delta = 0.02; % Safety margin
% n_lane_circles = 1;
% % [a1,b1,c1] = collision_check(p1,p2);
% % [a2,b2,c2] = collision_check(p3,p4);
% 
% for k = 1:N+1
%     for n = 1:N_seg
% %         g = [g; -abs(a1 *X(1,k) + b1*X(2,k) +c1)/(a1.^2 + b1.^2)^0.5 + (rob_diam/2 + delta);
%           g = [g; -sqrt((X(1,k)-coord(n,1))^2+(X(2,k)-coord(n,2))^2) + (rob_diam/2 + dia_lane_circ/2);];
%     end   
%           
% end

% -abs(a1 *X(1,k) + b1*X(2,k) +c1)/(a1.^2 + b1.^2)^0.5 + (rob_diam/2 + delta)
% -abs(a2 *X(1,k) + b2*X(2,k) +c2)/(a2.^2 + b2.^2)^0.5 + (rob_diam/2 + delta)


% make the decision variable one column  vector
OPT_variables = [reshape(X, n_states*(N+1),1);reshape(U,n_controls*N,1)];


nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0; %0,3
opts.print_time = 0;
opts.ipopt.linear_solver = 'spral';
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob, opts);

args = struct;
args.lbg(1:n_states*(N+1)) = 0; % equality constraints
args.ubg(1:n_states*(N+1)) = 0; % equality constraints

% args.lbg(n_states*(N+1)+1 : n_states*(N+1)) = -Inf; % inequality constraints
% args.ubg(n_states*(N+1)+1 : n_states*(N+1)) = 0; % inequality constraints


args.lbx(1:n_states:n_states*(N+1),1) = -5; %state x lower bound
args.ubx(1:n_states:n_states*(N+1),1) = 10; %state x upper bound
args.lbx(2:n_states:n_states*(N+1),1) = -5; %state y lower bound
args.ubx(2:n_states:n_states*(N+1),1) = 10; %state y upper bound
args.lbx(3:n_states:n_states*(N+1),1) = -1*pi; %state psi lower bound
args.ubx(3:n_states:n_states*(N+1),1) = +1*pi; %state psi upper bound
args.lbx(4:n_states:n_states*(N+1),1) =  0.6; %state v_x lower bound
args.ubx(4:n_states:n_states*(N+1),1) = 2; %state v_x upper bound
args.lbx(5:n_states:n_states*(N+1),1) = -0.2; %state v_y lower bound
args.ubx(5:n_states:n_states*(N+1),1) = 0.2; %state v_y upper bound
args.lbx(6:n_states:n_states*(N+1),1) = -2*pi; %state r lower bound
args.ubx(6:n_states:n_states*(N+1),1) = 2*pi; %state r upper bound
args.lbx(7:n_states:n_states*(N+1),1) = -2*pi; %state delta lower bound
args.ubx(7:n_states:n_states*(N+1),1) = 2*pi; %state delta upper bound



args.lbx(n_states*(N+1)+1:2:n_states*(N+1)+n_controls*N,1) = F_min; %v lower bound
args.ubx(n_states*(N+1)+1:2:n_states*(N+1)+n_controls*N,1) = F_max; %v upper bound
args.lbx(n_states*(N+1)+2:2:n_states*(N+1)+n_controls*N,1) = d_delta_min; %omega lower bound
args.ubx(n_states*(N+1)+2:2:n_states*(N+1)+n_controls*N,1) = d_delta_max; %omega upper bound
%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SET UP


% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------
t0 = 0;


xx(:,1) = x0; % xx contains the history of states
t(1) = t0;

u0 = zeros(N,2);        % two control inputs for each robot
X0 = repmat(x0,1,N+1)'; % initialization of the states decision variables

sim_tim = 10; % Maximum simulation time

% Start MPC:
mpciter = 0;
xx1 = [];
u_cl=[];

% the main simulaton loop... it works as long as the error is greater
% than 10^-6 and the number of mpc steps is less than its maximum
% value.
main_loop = tic;
tol = 4e-1;
obs_cl = [];

while(norm((x0(1:2)-xs(1:2)),2) > tol && mpciter < sim_tim / T)
    args.p(1:2*n_states)   = [x0;xs]; % set the values of the parameters vector
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
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    u = reshape(full(sol.x(n_states*(N+1)+1:end))',2,N)'; % get controls only from the solution
    xx1(:,1:7,mpciter+1)= reshape(full(sol.x(1:7*(N+1)))',7,N+1)'; % get solution TRAJECTORY
    u_cl= [u_cl ; u(1,:)];
    t(mpciter+1) = t0;

    % Apply the control and shift the solution
    [t0, x0, u0] = shift(T, t0, x0, u,f);
    xx(:, mpciter+2) = x0;
    X0 = reshape(full(sol.x(1:7*(N+1)))',7,N+1)'; % get solution TRAJECTORY
    % Shift trajectory to initialize the next step
    X0 = [X0(2:end,:); X0(end,:)];
    ss_error = norm((x0(1:2)-xs(1:2)),2)
    mpciter = mpciter + 1;
end

main_loop_time = toc(main_loop);
fprintf('Total elapsed time %f \n',main_loop_time)
average_mpc_time = main_loop_time/(mpciter+1)

path_length = 0;
for i = 2:size(xx,2)
    lane_segment = norm(xx(1:2,i)-xx(1:2,i-1));
    path_length  = path_length + lane_segment;
end
fprintf('Trajectory length : %f \n',path_length)

% figure
% surf(A,B,potential_field)
% f  = gcf;
% print(f,'/home/anshulnayak/MPC/Matlab/Total_potential_field.png','-dpng','-r300');

figure
subplot(7,1,1)
plot(t,squeeze(xx1(1,1,:)),'r','linewidth',1.5);
hold on
plot(t,xx(1,2:end),'b','linewidth',1);

subplot(7,1,2)
plot(t,squeeze(xx1(1,2,:)),'r','linewidth',1.5);
hold on
plot(t,xx(2,2:end),'b','linewidth',1);

subplot(7,1,3)
plot(t,squeeze(xx1(1,3,:)),'r','linewidth',1.5);
hold on
plot(t,xx(3,2:end),'b','linewidth',1);

subplot(7,1,4)
plot(t,squeeze(xx1(1,4,:)),'r','linewidth',1.5);
hold on
plot(t,xx(4,2:end),'b','linewidth',1);

subplot(7,1,5)
plot(t,squeeze(xx1(1,5,:)),'r','linewidth',1.5);
hold on
plot(t,xx(5,2:end),'b','linewidth',1);

subplot(7,1,6)
plot(t,squeeze(xx1(1,6,:)),'r','linewidth',1.5);
hold on
plot(t,xx(6,2:end),'b','linewidth',1);

subplot(7,1,7)
plot(t,squeeze(xx1(1,7,:)),'r','linewidth',1.5);
hold on
plot(t,xx(7,2:end),'b','linewidth',1);



Draw_MPC_APF_Obstacle_avoidance (t,xx,xx1,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_cl,obs_diam,mass, rx, ry,tol,factor_val)
% Draw_MPC_PS_Obstacles(t,xx,xx1,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_cl,obs_diam,mass, rx, ry,tol,factor_val)
















