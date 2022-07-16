function [dx] = CBRmodel_lagrangian(t,x)

global p;

q1 = x(1);
q2 = x(2);
q = [q1;q2];

dq1 = x(3);
dq2 = x(4);
dq = [dq1;dq2];

%Mass Matrix

m11 = @(q) (p.mH+p.m)*p.l^2 + p.m*p.a^2;
m12 = @(q) -p.m*p.l*p.b*cos(q(1)-q(2));
m21 = @(q) -p.m*p.l*p.b*cos(q(1)-q(2));
m22 = @(q) p.m*p.b^2;

%Coriolis Matrix 
c11 = @(q) 0;
c12 = @(q,dq) -p.m*p.l*p.b*sin(q(1) - q(2))*dq(2);
c21 = @(q,dq) p.m*p.l*p.b*sin(q(1) - q(2))*dq(1);
c22 = @(q) 0;

% Gravitational torque vector
g1 = @(q) -(p.mH*p.l + p.m*p.a + p.m*p.l)*p.g*sin(q(1));
g2 = @(q) p.m*p.b*p.g*sin(q(2));   

% Model
M = [m11(q), m12(q); m21(q), m22(q)];
p.M = M;
C = [c11(q), c12(q,dq); c21(q,dq), c22(q)];
g = [g1(q); g2(q)];
G = [1 0; 1 -1];

% Rotated gravity torque vector
g_psi = @(q) [g1(q+p.slope_change); g2(q+p.slope_change)];

if(p.controller == 1)
    % Passive
    u = [0;0]; 
elseif(p.controller == 2)
    % Control via symmetry
    % Passivity-Based Control of Bipedal Locomotion Regulating Walking by Exploiting 
    % Passive Gaits in 2-D and 3-D Bipeds 1070-9932/07/$25.00©2007 IEEEIEEE 
    % Robotics & Automation Magazine JUNE 200730
    u = G\(g-g_psi(q));
elseif(p.controller == 3)
    % Control via symmetry + PBC (enlarges the basin of attraction) 
    % Passivity-Based Control of Bipedal Locomotion Regulating Walking by Exploiting 
    % Passive Gaits in 2-D and 3-D Bipeds 1070-9932/07/$25.00©2007 IEEEIEEE 
    % Robotics & Automation Magazine JUNE 200730
    V = p.g*(p.m*(p.a+p.l)+p.mH*p.l)*cos(q1+p.slope_change) - p.g*p.b*p.m*cos(q2+p.slope_change);
    E = 0.5*dq'*M*dq + V;
    Eref = 153.0787; % Total energy for passive limit cycle
    k = 1;
    u_bar = -k*(E-Eref)*dq;
    u_psi = G\(g-g_psi(q));
    u = G\u_psi + G\u_bar;
elseif(p.controller == 4)
    % Control via feedback linearization
    % Feedback Control of Planar Biped Robot With Regulable
    % Step Length and Walking Speed Yong Hu, Gangfeng Yan, and Zhiyun Lin, Member, IEEE
    q_r  = [x(5);x(6)]; 
    dq_r = [x(7);x(8)];
    M_r = [m11(q_r), m12(q_r); m21(q_r), m22(q_r)];
    C_r = [c11(q_r), c12(q_r,dq_r); c21(q_r,dq_r), c22(q_r)];
    ddq_r = -M_r\(C_r*dq_r + g_psi(q_r));
    e_r = q_r - q;
    de_r = dq_r - dq;
    K1 = diag([10,10]);
    K2 = diag([25,25]);
    u = G\(M*ddq_r + C*dq + g) + G\M*(K1*de_r + K2*e_r);   
elseif(p.controller == 5)
    % Control via Energy pumping-and-damping
    % Energy pumping-and-damping for gait robustification of underactuated
    % planar biped robots within the hybrid zero dynamics framework
    V = p.g*(p.m*(p.a+p.l)+p.mH*p.l)*cos(q1+p.slope_change) - p.g*p.b*p.m*cos(q2+p.slope_change);
    P = M*dq; % Momentum
    H = 0.5*P.'/M*P + V;
    Href = 153.0787; % Total energy for passive limit cycle
    e = H - Href; % Output error variable
    % u = -Kpd*e*Gp.'*dHdp
    Kpd = diag([25 25]);
    Gp = G; % Input mapping matrix
    dHdp = M\P; % H = 0.5*P.'*\M*P + V(q), thus, dHdp = M\P
    u_psi = G\(g-g_psi(q)); % Potential Energy Shaping Control to make system slop invariant
    u_EPD = -Kpd*e*Gp.'*dHdp; % Energy Pumping-and-dumping control
    u = u_EPD + u_psi;
end
dq = [dq1;dq2];
ddq = M\(-C*dq-g+G*u);
dx = [dq; ddq];
if(p.controller == 4)
    dq_r = [x(7);x(8)];
    dx_r  = [dq_r; ddq_r];
    dx = [dx; dx_r];
end
end

