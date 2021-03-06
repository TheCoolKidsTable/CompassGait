function animate(q,psi,tci)
global p;
%% Record Figure
 writerObj = VideoWriter('passive_walk');
 writerObj.FrameRate = 60;
 open(writerObj);
%% Animate passive dynamic walking data
% Position of stance foot
rAAa = [0;0];
% Position of hip
rBAa = [rAAa(1) + p.l*sin(-q(1,1)); rAAa(2) + p.l*cos(-q(1,1))];
% Position of swing foot
rCBa = rBAa - [p.l*sin(-q(1,2)); p.l*cos(-q(1,2))];
% Initialize figure for animation
figure('Color','w','Renderer','zbuffer')
axis([0 8 -2 2])
set(gca,'nextplot','replacechildren');
set(gcf,'Renderer','zbuffer');
% Plot slope
R = @(slope) [cos(slope), sin(slope); -sin(slope), cos(slope)];
rPAa = R(p.psi)*[100;0]; % Point at end of slope rotated
slope = line([rAAa(1) rPAa(1)],[rAAa(2) rPAa(2)]);
set(slope,'Color','black','LineWidth',2)
% Draw first position
stleg = line([rAAa(1) rBAa(1)],[rAAa(2) rBAa(2)]);
set(stleg,'Color',[17 17 17]/255,'LineWidth',2);
swleg = line([rBAa(1) rCBa(1)],[rBAa(2) rCBa(2)]);
set(swleg,'Color','black','LineWidth',2);
hold on;
hip = plot(rBAa(1),rBAa(2),'o','MarkerSize',10,'MarkerFaceColor','blue');
grid on;
drawnow             % Force Matlab to draw
% % Animate each stride
tci = [0, tci];
% Position of stance foot
rAAa = [0;0];
for i = 1:length(tci)-1
    for j=tci(i)+1:tci(i+1)
        % Position of hip
        rBAa = [rAAa(1) + p.l*sin(-q(j,1)); rAAa(2) + p.l*cos(-q(j,1))];
        % Position of swing foot
        rCBa = rBAa - [p.l*sin(-q(j,2)); p.l*cos(-q(j,2))];
        % Clear previous legs
        set([stleg swleg hip],'Visible','off');
        % Plot new leg positions
        stleg = line([rAAa(1) rBAa(1)],[rAAa(2) rBAa(2)]);
        set(stleg,'Color','black','LineWidth',2);
        swleg = line([rBAa(1) rCBa(1)],[rBAa(2) rCBa(2)]);
        set(swleg,'Color','black','LineWidth',2);
        hold on;
        hip = plot(rBAa(1),rBAa(2),'o','MarkerSize',10,'MarkerFaceColor','white','MarkerEdgeColor','black','LineWidth',2);
        drawnow
        frame = getframe(gcf);
        writeVideo(writerObj,frame);
    end
    % Add History Plot
    if(mod(j,2) == 0)
        stleg_hist = line([rAAa(1) rBAa(1)],[rAAa(2) rBAa(2)]);
        set(stleg_hist,'Color',[0 0 0 0.2],'LineWidth',1,'LineStyle','--');
        swleg_hist = line([rBAa(1) rCBa(1)],[rBAa(2) rCBa(2)]);
        set(swleg_hist,'Color',[0 0 0 0.2],'LineWidth',1,'LineStyle','--');
        hold on;
        hip_hist = plot(rBAa(1),rBAa(2),'o','MarkerSize',10,'MarkerFaceColor','white','MarkerEdgeColor',[200 200 200]/255,'LineWidth',1);
    end
    % Update stance foot location
    rAAa = rCBa;
end
close(writerObj);
end