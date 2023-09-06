close all
clc
clear all

% Defining the obstacle location:

rx1 =1; ry1 = 1; 
rx2 =2;  ry2 =2;


% Safety around the obstacle:
d_0 = 0.1;

x = linspace(-3,3,100); nx = length(x);
y = linspace(-3,3,100); ny = length(y);

[X,Y] = meshgrid(x,y);

% Define the rectangles representing lane constraints for intersection:
rx = [-3, -1.5
         1.5, 3
        -3, -1.5 
        1.5, 3];
 
 ry = [-3,-1.5
          -3, -1.5
          1.5, 3
          1.5, 3];
      
 obs =[rx ry];
 n_obs = length(obs);

potential_field = zeros(nx,ny,n_obs);
dist = zeros(nx,ny,n_obs);
d1 =   zeros(nx,ny);
d2 =  zeros(nx,ny);

for k = 1:n_obs
    for i = 1:nx
        for j = 1:ny
        [ potential_field(i,j,k),  dist(i,j,k), d1(i,j), d2(i,j)] =  U_rep_pot(X(i,j),Y(i,j),obs(k,:), d_0);
        end
    end    
end
rep_potential = sum(potential_field,3);

figure
f  = gcf;
surf(X,Y,rep_potential)
print(f,'/home/anshulnayak/MPC/Matlab/Repulsive_Potential.png','-dpng','-r300');


% Computing the manhattan distance to the obstacle:
function [g,d,d1,d2] = U_rep_pot(x,y, obs, d_0)
    rx1 = obs(1);
    rx2 = obs(2);
    ry1 = obs(3);
    ry2 = obs(4);
    
    d1 = max([rx1- x,0, x-rx2]);
    d2 = max([ry1- y,0, y-ry2]);
    d = sqrt((max([rx1- x,0, x-rx2])).^2 + (max([ry1- y,0, y-ry2])).^2);
    g = 100 * sum(exp( -((d + d_0)^2/(2*d_0^2))));
    

end


