function agents = updateAgent(agents , pos_waypoint);

    for i = 1:length(agents)
        agents(i).goal = pos_waypoint(i,:);
    end