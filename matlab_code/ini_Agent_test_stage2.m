function agent = ini_Agent_test_stage2(pos_agent , pos_a_loop)

agent = [];
for i = 1 : length(pos_agent)
    agent = [agent ; struct( 'name', num2str(i), 'position', pos_agent(i,:), ...
        'velocity',[0 0], ...
        'goal',pos_a_loop(i,:), ...
        'path',[], ...
        'radius',100, ...
        'sensorRange',300, ...
        'vmax',10)];
end

end