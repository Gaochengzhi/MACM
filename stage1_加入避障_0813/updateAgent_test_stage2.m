function agents = updateAgent_test_stage2(agents , pos_a_loop);
    for i = 1:length(agents)
        agents(i).goal = pos_a_loop(i,:);
    end