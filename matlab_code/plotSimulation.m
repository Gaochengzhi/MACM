function plotSimulation(agents, pos_waypoint, counter, dt, axisLimits, colors, save)
% plotSimulation - Plots the positions of the agents and their paths
%
% Syntax: plotSimulation(agents, counter, dt, axisLimits, save)
%
    clf('reset')
    hold on
    plot(pos_waypoint(:,1),pos_waypoint(:,2),'ko','LineWidth',3,'MarkerSize',15, 'DisplayName', 'Waypoints')
    for i = 1:length(agents)
        faceColor = [0 170 255] / 255;
        lineColor = [135 135 135] / 255;
%         filledCircle(agents(i).position, agents(i).radius, 1000, faceColor);
        plot(agents(i).position(1), agents(i).position(2), '*', 'Color', colors(i,:), 'MarkerSize', 10, 'DisplayName', 'Agents');
        quiver(agents(i).position(1), agents(i).position(2), agents(i).velocity(1), agents(i).velocity(2), 'Color', [0 0 0]);
        plot(agents(i).path(:, 1), agents(i).path(:, 2), 'Color', lineColor);
        text(agents(i).position(1), agents(i).position(2), agents(i).name);
    end
    title([ 'Time: ' num2str(counter*dt) 's' ])
    legend(legendUnq(gca));
    set(get(gca, 'XLabel'), 'String', 'X [m]');
    set(get(gca, 'YLabel'), 'String', 'Y [m]');
%     axis(axisLimits)
%     axis equal
    xlim([0 7250]);
    ylim([0 7250]);
    hold off
    drawnow

    if save
        saveas(gcf, ['run/', num2str(counter,'%04.f'), '.png']);
    end
end