function agent = ini_Agent_test(pos_a , traj_points);

agent = [];
for i = 1 : length(pos_a)
    agent = [agent ; struct( 'name', num2str(i), 'position', pos_a(i,:), ...
        'velocity',[0 0], ...
        'goal',traj_points{i}(:,1)', ...
        'path',[], ...
        'radius',100, ...
        'sensorRange',300, ...
        'vmax',10)];
end

end