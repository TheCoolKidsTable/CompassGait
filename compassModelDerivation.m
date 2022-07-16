clear
clc
close all;
%% Define system in over-parameterised space
% Coordinates
syms q1 q2 x1 y1 x2 y2 x3 y3
q = [q1; q2; x1; y1; x2; y2; x3; y3];
% Mass matrix
syms m mH J1 J2
M = diag([J1 J2 m m mH mH m m]);
% Potential energy
syms g b a l
V =  g*(m*(a+l)+mH*l)*cos(q1) - g*b*m*cos(q2);
%% Construct the reduced system using constraints
% Separate coordinates into independant (theta) and dependant (x,y)
qh = [q1; q2;];
qb = [x1; y1; x2; y2; x3; y3];
% Define constraint and jacobian
fc = [a*sin(-q1); a*cos(-q1); l*sin(-q1); l*cos(-q1); 
      l*sin(-q1)+b*sin(q2); cos(-q1)-b*cos(q2)];
% Dimensions
ch = length(fc);
nq = length(q);
n = nq-ch;
dfcdqh = jacobian(fc,qh);
% Compute reduced mass matrix
Mh = simplify([eye(n), dfcdqh.']*M*[eye(n); dfcdqh]);
% Potential energy
Vh = subs(V,qb,fc);
%% Dynamics
syms p ph1 ph2 dq1 dq2 t
ph = [ph1; ph2];
dq = [dq1;dq2];
Hh = 1/2*(ph.'/Mh)*ph + Vh;
param = parameters;
param.J1 = 0;
param.J2 = 0;
params_sym = [m,mH,a,b,l,J1,J2,g];
params_num = [param.m,param.mH,param.a,param.b,param.l,param.J1,param.J2,param.g];
dynamics = simplify([zeros(n) eye(n);-eye(n) zeros(n)]*[jacobian(Hh,qh).';jacobian(Hh,ph).']);
%% Generate MATLAB function
dynamics_with_params=subs(dynamics,params_sym,params_num);
% CBRmodel_lag=matlabFunction(dynamics_with_params,'File','CBRmodel_lag','Vars',{t,[qh;dq]});
CBRmodel_hamil=matlabFunction(dynamics_with_params,'File','CBRmodel_hamiltonian','Vars',{t,[qh;ph]});
%% Impact Dynamics
% Add excessive coordinate z to swing foot
syms z1 z2 J3 J4
q = [z1; z2; q1; q2; x1; y1; x2; y2; x3; y3;];
% Mass matrix
M = diag([J1 J2 J3 J4 m m mH mH m m]);
%% Construct the reduced system using constraints
% Separate coordinates into independant (theta) and dependant (x,y)
x = [z1; z2; q1; q2];
qb = [x1; y1; x2; y2; x3; y3];
% Define constraint and jacobian
fc = [z1 + a*sin(-q1); z2 + a*cos(-q1); z1 + l*sin(-q1); z2 + l*cos(-q1); 
      z1 + l*sin(-q1)+b*sin(q2); z2 + l*cos(-q1)-b*cos(q2)];
% Dimensions
ch = length(fc);
nq = length(q);
nz = nq-ch;
dfcdqh = jacobian(fc,x);
% Compute reduced mass matrix according to (2.3.9)
Mx = simplify([eye(nz), dfcdqh.']*M*[eye(nz); dfcdqh]);
Mx = subs(Mx,[J1 J2 J3 J4],[0 0 0 0]);
%% Contact Jacobian Calculation
Y = [z1 + l*sin(-q1) + l*sin(q2); z2+l*cos(-q1)-l*cos(q2)]; %% Coordinates of swing foot
Jc = jacobian(Y,x).';
%% Lagrangian velocity mapping
R = fliplr(eye(n));
Px = (eye(4) - Mx\Jc/(Jc.'/Mx*Jc)*Jc.');
% Extract mapping for angular velocity
Pq = Px(n+1:end,n+1:end);
% Map to switching coordinates
Pq = R*Pq;
syms dq1 dq2
impact_map = [R, zeros(n);zeros(n), Pq]*[q1;q2;dq1;dq2];
impact_map_with_params=subs(impact_map,params_sym,params_num);
matlabFunction(impact_map_with_params,'File','impactMap_lagrangian','Vars',{[q1;q2;dq1;dq2]});
%% Hamiltonian momentum mapping
Px = (eye(4) - Jc/(Jc.'/Mx*Jc)*Jc.'/Mx);
% Extract mapping for momentum
Pq = Px(n+1:end,n+1:end);
% Map to switching coordinates
Pq = R*Pq;
syms p1 p2
impact_map = [R, zeros(n);zeros(n), Pq]*[q1;q2;p1;p2];
impact_map_with_params=subs(impact_map,params_sym,params_num);
matlabFunction(impact_map_with_params,'File','impactMap_hamiltonian','Vars',{[q1;q2;p1;p2]});
