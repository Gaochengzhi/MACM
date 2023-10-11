function agents = updateAgent_test(agents , traj_points);

    for i = 1:length(agents)
        if ~isempty(traj_points{i})
        agents(i).goal = traj_points{i}(:,1)';
        end
    end