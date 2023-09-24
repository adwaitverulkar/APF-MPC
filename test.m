% first casadi test for mpc fpr mobile robots
clear all
close all
clc

% CasADi v3.4.5
% addpath('C:\Users\mehre\OneDrive\Desktop\CasADi\casadi-windows-matlabR2016a-v3.4.5')
% CasADi v3.5.5
addpath('..\casadi-3.6.3-windows64-matlab2018b\')


import casadi.*

opti = Opti();

x = opti.variable();
y = opti.variable();
p = opti.parameter();

opti.minimize((y-x^2*p)^2)
opti.subject_to(x^2+y^2==1)
opti.subject_to(x+y>=1)

opti.solver('ipopt')

M = opti.to_function('M',{p},{[x; y]});

% https://www.youtube.com/watch?v=JI-AyLv68Xs


