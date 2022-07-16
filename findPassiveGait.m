clear;clc;close all;
%% Import urdf parser library
addpath('urdf2dynamics/');
addpath('urdf2dynamics/utility/');
sympref('FloatingPointOutput',true);
%% Model Parameters
global p;
p = parameters();
%% Define cost
x0 = [0.2187;-0.3234;-1.0918;-0.3772];
x0 = 0.9*x0;
% Cost
cost = @(x0) norm(x0 - stepSim(x0));
%% Run optimization to find initial conditions
options = optimoptions('fmincon','OptimalityTolerance', 1e-5, 'StepTolerance', 1e-3,'Algorithm','interior-point','MaxFunctionEvaluations',1e10);
A = [];
b = [];
Aeq = [];
beq = [];
lb = [-pi,-pi,-pi,-pi];
ub = [pi,pi,pi,pi];
x0_passive = fminunc(cost,x0,options);%,A,b,Aeq,beq,lb,ub); 

