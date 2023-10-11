function obs = addObs(name, position, velocity)
% addAgent - Return a struct with agent details
%
% Syntax: agent = addAgent(name, position, velocity, goal)
%
    obs = struct( 'name', name, 'position', position, 'velocity', velocity );
    obs.goal = [];
    obs.path = [];
    obs.radius = 0;
    obs.sensorRange = 0;
    obs.vmax = 0;%%最大速度
    obs.newControl = [];
end