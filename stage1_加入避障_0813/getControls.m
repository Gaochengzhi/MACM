function controls = getControls(agent, obstacles, dt, p_forbidArea, p_task)
%getControls - Returns the controls that will avoid collision
%
% Syntax: controls = getControls(agent, obstacles, dt)
%
    cost = @(u) desiredVelocityCost(agent, u') + 0.5*sum((agent.velocity - u').^2);
%     cost = @(u) desiredVelocityCost(agent, u');
    constraints = [];
%     if length(obstacles) > 0
        constraints = @(u) getConstraints(agent, obstacles, u', dt, p_forbidArea, p_task);
%     end
    init = agent.velocity' * 0.5;
%     lb = [-1.2 -1.2];
%     ub = [1.2 1.2];
    lb = [-60 -60];%%速度下限
    ub = [60 60];%%速度上限
    options = optimoptions('fmincon','Display','final-detailed','Algorithm','sqp');
    controls = fmincon(cost, init, [], [], [], [], lb, ub, constraints, options)';

    % Smoothen the controls
    controls = smoothenControls(agent, controls);
end