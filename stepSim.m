function xplus = stepSim(x0)
    %% Run ODE solver with event
    t_sim = [0 5];
    options = odeset('Events',@impactEvent);
    % Halt and return position of ball if it intersects the point.
    [t,x,te,ye,ie] = ode45(@CBRmodel_lagrangian,t_sim,x0,options);
    % Apply impact mapping
    xminus = x(end,:).';
    xplus = impactMap_lagrangian(xminus);
end