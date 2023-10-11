function [c ceq] = getConstraints(agent, obstacles, control, dt, p_forbidArea, p_task)
%getConstraints - Description
%
% Syntax: [c ceq] = getConstraints(agent, obstacles)
%
c = [];
ceq = [];

% Time horizon
tau = 1000;

for i = 1:length(obstacles)
    % Refer the paper for explanation on these terms
    %         vRel = agent.velocity - obstacles(i).velocity;
    %         vAb = agent.velocity + obstacles(i).velocity;
    %         pAb = (agent.position - obstacles(i).position) / tau;
    %
    %         % Finding pAb perpendicular
    %         r = 2 / tau;
    %         l = abs(sqrt(sum(pAb.^2) - r^2));
    %         m = [
    %             l -r;
    %             r  l
    %         ];
    %         qL = (pAb * m') * (l / sum(pAb.^2));
    %         qR = (pAb * m ) * (l / sum(pAb.^2));
    %         pAbL = [qL(2) -qL(1)];
    %         pAbR = [qR(2) -qR(1)];
    %
    %         c(end+1) = -1*(2*control(1)*(pAbR(1)) + 2*control(2)*(pAbR(2)) - (pAbR(1))*(agent.velocity(1)) - (pAbR(2))*(agent.velocity(2)) - (pAbR(1))*(obstacles(i).velocity(1)) - (pAbR(2))*(obstacles(i).velocity(2)));
    %
    %         c(end+1) = -1*(2*control(1)*(pAbL(1)) + 2*control(2)*(pAbL(2)) - (pAbL(1))*(agent.velocity(1)) - (pAbL(2))*(agent.velocity(2)) - (pAbL(1))*(obstacles(i).velocity(1)) - (pAbL(2))*(obstacles(i).velocity(2)));

    %%%%%%%%%%% consider the threaten range %%%%%%%%%%%%%%%
    c(end+1) = -norm(agent.position + control*dt - obstacles(i).position) + agent.radius;

    % Choosing the constrain based on the same side contraint
    % Refer paper for more details
    %         if det([pAb; vRel]) < 0
    %             c(end+1) = -1*(control(1)*(pAbR(1))^2*(agent.velocity(1)) + control(2)*(pAbR(1))*(pAbR(2))*(agent.velocity(1)) - 0.5*(pAbR(1))^2*(agent.velocity(1))^2 + control(1)*(pAbR(1))*(pAbR(2))*(agent.velocity(2)) + control(2)*(pAbR(2))^2*(agent.velocity(2)) - 1.*(pAbR(1))*(pAbR(2))*(agent.velocity(1))*(agent.velocity(2)) - 0.5*(pAbR(2))^2*(agent.velocity(2))^2 - control(1)*(pAbR(1))^2*(obstacles(i).velocity(1)) - control(2)*(pAbR(1))*(pAbR(2))*(obstacles(i).velocity(1)) + 0.5*(pAbR(1))^2*(obstacles(i).velocity(1))^2 - control(1)*(pAbR(1))*(pAbR(2))*(obstacles(i).velocity(2)) - control(2)*(pAbR(2))^2*(obstacles(i).velocity(2)) + 1.*(pAbR(1))*(pAbR(2))*(obstacles(i).velocity(1))*(obstacles(i).velocity(2)) + 0.5*(pAbR(2))^2*(obstacles(i).velocity(2))^2);
    %         else
    %             c(end+1) = -1*(control(1)*(pAbL(1))^2*(agent.velocity(1)) + control(2)*(pAbL(1))*(pAbL(2))*(agent.velocity(1)) - 0.5*(pAbL(1))^2*(agent.velocity(1))^2 + control(1)*(pAbL(1))*(pAbL(2))*(agent.velocity(2)) + control(2)*(pAbL(2))^2*(agent.velocity(2)) - 1.*(pAbL(1))*(pAbL(2))*(agent.velocity(1))*(agent.velocity(2)) - 0.5*(pAbL(2))^2*(agent.velocity(2))^2 - control(1)*(pAbL(1))^2*(obstacles(i).velocity(1)) - control(2)*(pAbL(1))*(pAbL(2))*(obstacles(i).velocity(1)) + 0.5*(pAbL(1))^2*(obstacles(i).velocity(1))^2 - control(1)*(pAbL(1))*(pAbL(2))*(obstacles(i).velocity(2)) - control(2)*(pAbL(2))^2*(obstacles(i).velocity(2)) + 1.*(pAbL(1))*(pAbL(2))*(obstacles(i).velocity(1))*(obstacles(i).velocity(2)) + 0.5*(pAbL(2))^2*(obstacles(i).velocity(2))^2);
    %         end
end

for i = 1 : length(p_forbidArea)
size_forbid = polyarea(p_forbidArea{1,i}(1,:),p_forbidArea{1,i}(2,:));
rectan_forbid = size_rectan(agent.position + control*dt , p_forbidArea{1,i});
% size_area = polyarea(p_task(1,:),p_task(2,:));
% rectan_area = size_rectan(agent.position + control*dt , p_task);
c(end+1) = - (rectan_forbid - size_forbid) + 0.08*size_forbid;
end
% c(end+1) = rectan_area - size_area;
c(end+1) = - norm(control) + agent.vmax;

% Kinematic constraints
% delVel = agent.velocity - control;
% c(end+1) = abs(atan2(delVel(2), delVel(1))) - pi/20;
end