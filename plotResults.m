function [] = plotResults(t,x,tci)
global p;
%% States
figure;
subplot(2,2,1)
plot(t,x(:,1),'LineWidth',2);
if(p.controller == 4)
    hold on;
    plot(t,x(:,5),'--','LineWidth',2);
    legend('q_1','q_1 reference')
end
title('$q_1$','interpreter','latex')
ylabel('q_1 [rad]')
xlabel('time [s]')
grid on;

subplot(2,2,2)
plot(t,x(:,2),'LineWidth',2);
if(p.controller == 4)
    hold on;
    plot(t,x(:,6),'--','LineWidth',2);
    legend('q_2','q_2 reference')
end
title('$q_2$','interpreter','latex')
ylabel('q_2 [rad]')
xlabel('time [s]')
grid on;

subplot(2,2,3);
plot(t,x(:,3),'LineWidth',2);
if(p.controller == 4)
    hold on;
    plot(t,x(:,7),'--','LineWidth',2);
    legend('$\dot{q_1}$ [rad/s]','$\dot{q_1}$ [rad/s] reference','interpreter','latex')
end
title('$\dot{q_1}$','interpreter','latex')
ylabel('$\dot{q_1}$ [rad/s]','interpreter','latex')
xlabel('time [s]')
grid on;

subplot(2,2,4);
plot(t,x(:,4),'LineWidth',2);
if(p.controller == 4)
    hold on;
    plot(t,x(:,8),'--','LineWidth',2);
    legend('$\dot{q_2}$ [rad/s]','$\dot{q_2}$ [rad/s] reference','interpreter','latex')
end
title('$\dot{q_2}$','interpreter','latex')
ylabel('$\dot{q_2}$ [rad/s]','interpreter','latex')
xlabel('time [s]')
grid on;

%% Limit cycle
figure;
hold on;
plot(x(:,1),x(:,3),'.','LineWidth',2);
plot(x(:,2),x(:,4),'.','LineWidth',2);
% plot([x(1,1) x(end,2)], [x(1,3) x(end,4)],'LineWidth',2,'Color','black')
% plot([x(1,2) x(end,1)], [x(1,4) x(end,3)],'LineWidth',2,'Color','black')
title('limit cycle')
ylabel('joint velocity [rad/s]')
xlabel('joint angle [rad]')
legend('\theta_1','\theta_2','Impacts')
grid on;

%% Energy
T = []; % Kinetic Energy
V = []; % Potential Energy
P = []; % Momentum
%Mass Matrix
m11 = @(q) (p.mH+p.m)*p.l^2 + p.m*p.a^2;
m12 = @(q) -p.m*p.l*p.b*cos(q(1)-q(2));
m21 = @(q) -p.m*p.l*p.b*cos(q(1)-q(2));
m22 = @(q) p.m*p.b^2;
for i=1:length(x)
    q = x(i,1:2).';
    dq = x(i,3:4).';
    M = [m11(q), m12(q); m21(q), m22(q)];
    T = [T, 0.5*dq'*M*dq]; % Append Kinetic Energy to vector
    V = [V, p.g*(p.m*(p.a+p.l)+p.mH*p.l)*cos(q(1)) - p.g*p.b*p.m*cos(q(2))]; % Append Potential Energy to vector
    P = [P, M*dq];
end

figure;
subplot(1,2,1);
plot(t,V,t,T,'LineWidth',2);
legend('Potential Energy [J]', 'Kinetic Energy [J]');
title('Potential and Kinetic Energy');
ylabel('Energy [J]');
xlabel('Time [s]');
grid on;
subplot(1,2,2);
H = T + V;
plot(t,H,'LineWidth',2);
legend('Hamiltonian (Total Energy) [J]')
title('Total Energy');
ylabel('Energy [J]');
xlabel('Time [s]');
axis([0 t(end) 150 160])
grid on;
figure;
subplot(1,2,1);
plot(t,P,'LineWidth',2);
legend('Momentum P1','Momentum P2');
title('Momentum');
ylabel('Momentum');
xlabel('Time [s]');
grid on;
subplot(1,2,2);
plot(t,P(1,:)+P(2,:),'LineWidth',2);
legend('Momentum Total');
title('Momentum');
ylabel('Momentum');
xlabel('Time [s]');
grid on;
%% Plot angular momentum over time about the stance foot
L_stance = zeros(length(x),3);

flag = -1;
j = 1;
for i=1:length(x)
    q = x(i,1:2).';
    dq = x(i,3:4).';

    if(tci(j) == i)
        flag=flag*-1;
        j=j+1;
    end

    r200 = [-p.b*sin(q(2));p.b*cos(q(2));0];

    rH00 = [-p.l*sin(-q(1));p.l*cos(-q(1));0];

    r100 = rH00 + [-p.a*sin(-q(1));-p.a*cos(-q(1));0];
    
    
    
    if(flag == -1)
        vH00 = cross([0;0;dq(1)],[p.l*sin(-q(1));p.l*cos(-q(1)); 0]);
        v100 = cross([0;0;dq(1)],[p.b*sin(-q(1));p.b*cos(-q(1));0]);
        v200 = vH00+cross([0;0;dq(2)],[p.a*sin(q(2));-p.a*cos(q(2));0]);
    else
        vH00 = cross([0;0;dq(2)],[-p.l*sin(q(2));p.l*cos(q(2));0]);
        v100 = vH00 + cross([0;0;dq(1)],[-p.a*sin(-q(1));-p.a*cos(-q(1));0]);
        v200 = cross([0;0;dq(2)],[-p.b*sin(q(2));p.b*cos(q(2));0]);
    end
    L_stance(i,:) = cross(r100,p.m*v100) + cross(rH00,p.mH*vH00) + cross(r200,p.m*v200);
end

figure;
plot(L_stance)


end

