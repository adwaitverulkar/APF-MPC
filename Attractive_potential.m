clc
clear 

% First Attractive potential function based on Logistic Growth:
% f(x) = L/(1 + exp(-k(x -x_0))
x_0 = [1,2];
L = 10;
k = 0.2;

% Code for APF-MPC
% Second Attractive potential based on linear + quadratic:
K_att = 10;
d_0 = 0.3;

x = linspace(-3,3,100); nx = length(x);
y = linspace(-3,3,100); ny = length(y);

[X,Y] = meshgrid(x,y);

U_att1 = zeros(nx,ny);
U_att2 = U_att1;

for i = 1:nx
    for j = 1:ny
        U_att1(i,j) = L/(1+exp(-k* sqrt((X(i,j)-x_0(1)).^2 + (Y(i,j)-x_0(2)).^2)));
        
        d = sqrt((X(i,j)-x_0(1)).^2 + (Y(i,j)-x_0(2)).^2);
        if (d <= d_0)
            U_att2(i,j) = 0.5 * K_att * d.^2;
        else
            U_att2(i,j) = -0.5*K_att*d_0.^2 + d_0*K_att*d;
        end
    end
end



figure
f  = gcf;
surf(X,Y,U_att1)
print(f,'/home/anshulnayak/MPC/Matlab/Attractive_Potential.png','-dpng','-r300');


% hold on
% surf(X,Y,U_att1)