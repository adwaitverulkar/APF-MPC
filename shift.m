function [t0, x0, u0] = shift(T, t0, x0, u,f)
% add noise to the control actions before applying it
con_cov = diag([0.005 deg2rad(2)]).^2;
con = u(1,:)' + sqrt(con_cov)*randn(2,1); 
st = x0;

% Runge - Kutta 4th order:
k1 = f(st, con);
k2 = f(st + 0.5*T*k1, con);
k3 = f(st + 0.5*T*k2, con);
k4 = f(st + T*k3, con);
st_next_RK4 = st + T/6 *(k1 + 2*k2 + 2*k3 +k4);

% Euler integration: 
% f_value = f(st,con);   
% st = st+ (T*f_value);

x0 = full(st_next_RK4);
t0 = t0 + T;

u0 = [u(2:size(u,1),:);u(size(u,1),:)]; % shift the control action 
end