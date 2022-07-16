clc;
close all;
clear;
%% Model Parameters
global p;
p = parameters();
%% Simulation Parameters
total_sim_time = 10;
tspan = [0 total_sim_time]; 
state_space = [];
time = [];
tci = [];
current_time = 0;
step_count = 0;
max_steps = 15;
% Initial conditions
x0 = p.ic;
if(p.controller == 4)
    x0 = [x0;[0.2187-p.slope_change;-0.3234-p.slope_change;-1.0918;-0.3772]];
end
% Swing phase model
swingPhaseModel = @CBRmodel_lagrangian;
% Impact map
impactMapping = @(x) impactMap_lagrangian(x);
% Impact event
impactEventDetection = @impactEvent;

%% Simulation
while(current_time < total_sim_time ) && (step_count < max_steps)
    options = odeset('Events', impactEventDetection,'RelTol',1e-5,'AbsTol',1e-5);
    [t, x, event_time, event_state, event_id] = ode45(swingPhaseModel,tspan,x0,options);
    % Apply impact mapping
    x0 = impactMapping(x(end,:)');
    if(p.controller == 4)
        x0 = [x0; [0.2187-p.slope_change;-0.3234-p.slope_change;-1.0918;-0.3772]];
    end
    fprintf('Swing foot impact at time = %0.2f\n', t(end));
    time = [time; t];
    state_space = [state_space; x];
    tci = [tci length(time)];
    tspan = [t(end) max(total_sim_time, time(end))];
    current_time = time(end);
    step_count = step_count + 1;
end
%% Plotting
plotResults(time,state_space,tci);
%% Animation
if(p.animate)
    animate(state_space,p.psi,tci);
end