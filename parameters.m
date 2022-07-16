function [p] = parameters()
%% Model params
p.mH = 10; % Mass of hip [kg]
p.m = 5; % Mass of link [kg]
p.a = 0.5; % Length of lower leg [m]
p.b = 0.5; % Length of upper leg [m]
p.l = p.a + p.b; % Leg length [m]
p.g = 9.8; % Gravity [m/sˆ2]
%% Incline
p.slope_change = (-3/180)*pi; % Change the slope 
p.psi = (3/180)*pi+p.slope_change; % Incline of the slope
%% Initial conditions
p.initial_error = 1;
p.ic = p.initial_error*[0.2187-p.slope_change;-0.3234-p.slope_change;-1.0918;-0.3772];
%% Controller Type
% 1 = passive, 2 = control via symmetry, 3 = control via symmetry + PBC,
% 4 = feedback linearization, 5 = EPD-PBC
p.controller = 3; 
%% Animation
p.animate = true;