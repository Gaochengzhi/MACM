%% Clear workspace
clc;
clear all;
close all;

%% 导入数据
vesPostion = [121.65377598, 38.82595636;
    121.65377598, 38.82305520;
    121.65377598, 38.81996062;
    121.65377598, 38.81709814;
    121.65377598, 38.81431302;
    121.65377598, 38.81152791;
    121.65377598, 38.80847201;
    121.65377598, 38.80557085;];

vesPostion_3 = zeros(size(vesPostion,1),3);
vesPostion_3(:,1:2) = vesPostion(:,[2,1]);
p_agent = lla2ecef(vesPostion_3, 'WGS84');
p_agent(:,3) = [];

targets_pos_value = [121.66111478389999, 38.805913105872833;
    121.71429605931525, 38.816378517234796;
    121.66970109072057, 38.852307347659718;
    121.68604831104761, 38.845092037001393;
    121.6707626024872, 38.776710579528583;
    121.69921100930935, 38.780974165685052;
    121.69708799459914, 38.803767989386017;
    121.66078442629077, 38.827873681556639;
    121.69814950636578, 38.831973290969444;
    121.67840545897317, 38.816230794368096;
    121.68243918339334, 38.791305184403939;
    121.71322291874901, 38.794256902090751;
    121.67522093249633, 38.835580942891085;
    121.68286379339384, 38.782286047784794;
    121.69475267930022, 38.821314311948186;
    121.70112172343084, 38.849519613531612;
    121.69117078844744, 38.772996503899321;
    121.66524676197139, 38.790601531239759;
    121.6795136756341, 38.803099747183978;
    121.69726032670366, 38.793423708593934;
    121.70839548464096, 38.838847345176987;
    121.66663865781643, 38.816269912713494;
    121.7002181004114, 38.812372621474879;
    121.67812178420058, 38.826617900888834;
    121.71204920406547, 38.80605631718123;
    121.66502627357841, 38.84108488727361;
    121.71622402252871, 38.825370693544812;
    121.66338629520476, 38.784079916968153;
    121.70796813066727, 38.782007407946821;
    121.68732838777908, 38.810225433215066;
    121.70404657828328, 38.824095299183966;
    121.65781356356618, 38.821225675502149;
    121.68774118369561, 38.83589266976162;
    121.68487397874952, 38.853679222580809;
    121.67142315889046, 38.798675571360093;];

targets_pos_value_3 = zeros(size(targets_pos_value,1),3);
targets_pos_value_3(:,1:2) = targets_pos_value(:,[2,1]);
p_targets = lla2ecef(targets_pos_value_3, 'WGS84');
p_targets(:,3) = [];

%随机生成若干个禁航区
selected_targets = [targets_pos_value(10,:);
              targets_pos_value(22,:);
              targets_pos_value(8,:);
              targets_pos_value(26,:);
              targets_pos_value(13,:);
              targets_pos_value(24,:);
              targets_pos_value(10,:);];
forbidArea = (selected_targets(2:end,:) + selected_targets(1:end-1,:))/2;
forbidArea(end+1,:) = forbidArea(1,:);
forbidArea_3 = zeros(size(forbidArea,1),3);
forbidArea_3(:,1:2) = forbidArea(:,[2,1]);
p_forbidArea = lla2ecef(forbidArea_3, 'WGS84');
p_forbidArea(:,3) = [];


taskArea = [121.66473222 , 38.85573388;
    121.71089122 , 38.85573388;
    121.71940282 , 38.82943606;
    121.71940282 , 38.79479375
    121.70958175 , 38.76925451
    121.66505958 , 38.76925451
    121.65572957 , 38.79441445
    121.65572957 , 38.82791887
    121.66473222 , 38.85573388;];

taskArea_3 = zeros(size(taskArea,1),3);
taskArea_3(:,1:2) = taskArea(:,[2,1]);
p_task = lla2ecef(taskArea_3, 'WGS84');
p_task(:,3) = [];

p_all = [p_agent ; p_targets ; p_task ; p_forbidArea];

deviation_x = min(p_all(:,1));
deviation_y = min(p_all(:,2));

p_agent(:,1) = p_agent(:,1) - deviation_x;
p_agent(:,2) = p_agent(:,2) - deviation_y;
pos_a = p_agent;

p_targets(:,1) = p_targets(:,1) - deviation_x;
p_targets(:,2) = p_targets(:,2) - deviation_y;
pos_t = p_targets;


p_task = p_task';
p_task(1,:) = p_task(1,:) - deviation_x;
p_task(2,:) = p_task(2,:) - deviation_y;

p_forbidArea = p_forbidArea';
p_forbidArea(1,:) = p_forbidArea(1,:) - deviation_x;
p_forbidArea(2,:) = p_forbidArea(2,:) - deviation_y;

targets_angle = [60,210,105,135,30,240,120,165,15,45,75,330,60,255,180,225,285,120,75,150,300,90,270,285,240,180,255,60,240,150,300,90,270,285,240]';

targets_restCheck = [3,4,4,5,4,3,3,4,5,4,3,4,5,4,3,5,4,5,4,4,5,4,5,4,5,3,4,5,4,4,5,4,5,4,5]';

x = p_task(1,:)';
y = p_task(2,:)';
[lmin,lmax,V,laneDist] = findStrips(x,y,0,400,400);
lmin = sortrows(lmin,1);
lmax = sortrows(lmax,1);
pos_waypoints = [lmin;lmax];

p_GCAA = mat2cell([15   13	 11	 9	 7	 5	 3	 1],1,[1 1 1 1 1 1 1 1]);
% p_GCAA = mat2cell([31   29	 27	 25	 23	 21	 19	 17],1,[1 1 1 1 1 1 1 1]);
ind_completed = [];
targets_searched = [];

obs = [];
for i = 1 : length(pos_t)
    obs = [obs ; addObs(num2str(i), pos_t(i,:), [0 0])];
end
%% Declare the agents
% Usage: addAgent(name, initialPosition, initialVelocity, goal)
% agents = [
%     addAgent('1', pos_a(1,:), [0 0], pos_waypoints(15,:)),
%     addAgent('2', pos_a(2,:), [0 0], pos_waypoints(13,:)),
%     addAgent('3', pos_a(3,:), [0 0], pos_waypoints(11,:)),
%     addAgent('4', pos_a(4,:), [0 0], pos_waypoints(9,:)),
%     addAgent('5', pos_a(5,:), [0 0], pos_waypoints(7,:)),
%     addAgent('6', pos_a(6,:), [0 0], pos_waypoints(5,:)),
%     addAgent('7', pos_a(7,:), [0 0], pos_waypoints(3,:)),
%     addAgent('8', pos_a(8,:), [0 0], pos_waypoints(1,:))
% ];
% agents = [
%     addAgent('1', pos_waypoints(15,:), [0 0], pos_waypoints(31,:)),
%     addAgent('2', pos_waypoints(13,:), [0 0], pos_waypoints(29,:)),
%     addAgent('3', pos_waypoints(11,:), [0 0], pos_waypoints(27,:)),
%     addAgent('4', pos_waypoints(9,:), [0 0], pos_waypoints(25,:)),
%     addAgent('5', pos_waypoints(7,:), [0 0], pos_waypoints(23,:)),
%     addAgent('6', pos_waypoints(5,:), [0 0], pos_waypoints(21,:)),
%     addAgent('7', pos_waypoints(3,:), [0 0], pos_waypoints(19,:)),
%     addAgent('8', pos_waypoints(1,:), [0 0], pos_waypoints(17,:)),
%     ];
mat_p_GCAA = cell2mat(p_GCAA);
mat_p_GCAA(find(mat_p_GCAA == 0))=[];
pos_waypoint = pos_waypoints(mat_p_GCAA,:);
agents = addAgent(pos_a , pos_waypoint);

colors = lines(length(agents));
dynamic_flag = zeros(length(agents),1);
% agents = [
%     addAgent('1', [-5 -5], [0 0], [5 5]),
%     addAgent('2', [5 5],   [0 0], [-5 -5]),
%     addAgent('3', [-5 5],  [0 0], [5 -5]),
%     addAgent('4', [5 -5],  [0 0], [-5 5])
% ];
% axisLimits = [-6 6 -6 6]; % [xmin xmax ymin ymax] axis limits of the plot
dt = 3;
%% Simulation loop
% Runs till the distance to goal of all the agents is less than 0.32m
% Or for max iterations
maxIterations = 1500;
counter = 0;

while ~isempty(mat_p_GCAA)
    completed_tasks = [];
    maxDistFromGoal = 0;
    % Get the new velocity command for every agent but do not update it now
    for i = 1:length(agents)
        % Get agents in the sensor range
        %         obstacles = [addAgent('9', pos_t(1,:), [0 0], pos_t(1,:))];
        obstacles = [];
        for j = 1:length(agents)
            if i ~= j
                if inSensorRange(agents(i), agents(j))
                    obstacles = [obstacles; agents(j)];
                end
            end
        end

        for j=1:length(obs)
            if inSensorRange(agents(i), obs(j))
                obstacles = [obstacles; obs(j)];
                if isempty(find(targets_searched==obs(j).position(1,1)))
                    targets_searched = [targets_searched;obs(j).position];
                end
            end
        end
        % Get the new control for the agent
        agents(i).newControl = getControls(agents(i), obstacles, dt, p_forbidArea);
    end

    % Update the positions of all the agents using the newly obtained controls
    % This is equivalent to running the same algorithm simultaneously on all agents
    for i = 1:length(agents)
        agents(i).path = [agents(i).path; agents(i).position];
        agents(i).position = futurePosition(agents(i), dt);
        pos_agent(i,:) = agents(i).position;
        agents(i).velocity = agents(i).newControl;
        maxDistFromGoal = max(maxDistFromGoal, sum((agents(i).position - agents(i).goal).^2));
        if  norm(agents(i).position - agents(i).goal) <= 20 && isempty(find(completed_tasks == p_GCAA{i}))
            completed_tasks = [completed_tasks p_GCAA{i}];
        end
    end

    if ~isempty(completed_tasks) && length(pos_waypoint) == length(agents)
        for k = 1 : size(completed_tasks,2)
            ind_completed = find(cell2mat(p_GCAA) == completed_tasks(k));
            if ~isempty(ind_completed)
                if dynamic_flag(ind_completed)==0
                    p_GCAA{1,ind_completed} = p_GCAA{1,ind_completed}+size(lmin,1);
                    dynamic_flag(ind_completed) = dynamic_flag(ind_completed)+1;
                elseif dynamic_flag(ind_completed)==1
                    p_GCAA{1,ind_completed} = p_GCAA{1,ind_completed}+1;
                    dynamic_flag(ind_completed) = dynamic_flag(ind_completed)+1;
                elseif dynamic_flag(ind_completed)==2
                    p_GCAA{1,ind_completed} = p_GCAA{1,ind_completed}-size(lmin,1);
                    dynamic_flag(ind_completed) = dynamic_flag(ind_completed)+1;
                    %             elseif dynamic_flag(ind_completed)==3
                    %                 p_GCAA{1,ind_completed} = 0;
                end
            end
        end
    end

    if ~isempty(completed_tasks)
        for k = 1 : size(completed_tasks,2)
            ind_completed = find(cell2mat(p_GCAA) == completed_tasks(k));
            if dynamic_flag(ind_completed)==3
                for l = 1 : length(ind_completed)
                p_GCAA{1,ind_completed(l)} = 0;
                end
            end
        end
    end

    mat_p_GCAA = cell2mat(p_GCAA);
    num = find(mat_p_GCAA == 0);
    mat_p_GCAA(num)=[];
    pos_waypoint = pos_waypoints(mat_p_GCAA,:);
    if length(pos_waypoint) == length(agents)
        agents = updateAgent(agents , pos_waypoint);
    end


    % Plot the current simulation step
    % Usage: plotSimulation(agents, counter, dt, axisLimits, true) -> Save the outputs to disk
    %        plotSimulation(agents, counter, dt, axisLimits, false) -> Don't save the outputs to disk

    clf('reset')
    hold on
    plot(p_task(1,:),p_task(2,:),'k','DisplayName', 'Boundary','LineWidth',2);
    plot(p_forbidArea(1,:),p_forbidArea(2,:),'r','DisplayName', 'ForbidArea','LineWidth',2);
    plot(pos_waypoint(:,1),pos_waypoint(:,2),'ko','LineWidth',3,'MarkerSize',15, 'DisplayName', 'Waypoints')

    for i = 1:length(agents)
        faceColor = [0 170 255] / 255;
        lineColor = [135 135 135] / 255;
        %         filledCircle(agents(i).position, agents(i).radius, 1000, faceColor);
        plot(agents(i).position(1), agents(i).position(2), '*', 'Color', colors(i,:), 'MarkerSize', 10, 'DisplayName', 'Agents');
        PlotTaskLoitering_stage1(agents(i).position, agents(i).radius, 1, 'g', 'Agent Threaten Range');
        quiver(agents(i).position(1), agents(i).position(2), agents(i).velocity(1), agents(i).velocity(2), 'Color', [0 0 0]);
        plot(agents(i).path(:, 1), agents(i).path(:, 2), 'Color', lineColor);
        text(agents(i).position(1), agents(i).position(2), agents(i).name);
    end

    %     for i=1:length(obs)
    %         plot(obs(i).position(1), obs(i).position(2),'rs', 'MarkerSize', 10, 'DisplayName', 'Targets', 'MarkerFaceColor',[1 .6 .6])
    %         PlotTaskLoitering(obs(i).position, 50, 1, 'r', 'Target Threaten Range');
    %         text(obs(i).position(1), obs(i).position(2), obs(i).name);
    %     end
    if ~isempty(targets_searched)
        for i=1:size(targets_searched,1)
            plot(targets_searched(i,1), targets_searched(i,2),'rs', 'MarkerSize', 10, 'DisplayName', 'Targets', 'MarkerFaceColor',[1 .6 .6])
            num = find(pos_t == targets_searched(i,1));
            text(targets_searched(i,1), targets_searched(i,2),num2str(num),'FontSize',14);
            PlotTaskLoitering_stage1(targets_searched, 50, 1, 'r', 'Target Threaten Range');
            %         text(targets_searched(i).position(1), targets_searched(i).position(2), targets_searched(i).name);
        end
    end

    title('Stage 1: Area Coverage and Targets Searched:',num2str(size(targets_searched,1)),'FontSize',20);
    legend(legendUnq(gca));
    set(get(gca, 'XLabel'), 'String', 'X [m]');
    set(get(gca, 'YLabel'), 'String', 'Y [m]');
    %     axis(axisLimits)
    %     axis equal
    xlim([0 7250]);
    ylim([0 7250]);
    hold off
    drawnow

    counter = counter + 1;

    % Stop running of the goal is reached
%     if maxDistFromGoal < 0.1
%         break
%     end
end