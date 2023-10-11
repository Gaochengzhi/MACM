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
selected_targets1 = [targets_pos_value(10,:);
    targets_pos_value(22,:);
    targets_pos_value(8,:);
    targets_pos_value(26,:);
    targets_pos_value(13,:);
    targets_pos_value(24,:);
    targets_pos_value(10,:);];
forbidArea1 = (selected_targets1(2:end,:) + selected_targets1(1:end-1,:))/2;
forbidArea1(end+1,:) = forbidArea1(1,:);
forbidArea1_3 = zeros(size(forbidArea1,1),3);
forbidArea1_3(:,1:2) = forbidArea1(:,[2,1]);
p_forbidArea1 = lla2ecef(forbidArea1_3, 'WGS84');
p_forbidArea1(:,3) = [];

selected_targets2 = [targets_pos_value(11,:);
    targets_pos_value(19,:);
    targets_pos_value(30,:);
    targets_pos_value(7,:);
    targets_pos_value(20,:);];
forbidArea2 = (selected_targets2(2:end,:) + selected_targets2(1:end-1,:))/2;
forbidArea2(end+1,:) = forbidArea2(1,:);
forbidArea2_3 = zeros(size(forbidArea2,1),3);
forbidArea2_3(:,1:2) = forbidArea2(:,[2,1]);
p_forbidArea2 = lla2ecef(forbidArea2_3, 'WGS84');
p_forbidArea2(:,3) = [];

selected_targets3 = [targets_pos_value(15,:);
    targets_pos_value(9,:);
    targets_pos_value(21,:);
    targets_pos_value(31,:);];
forbidArea3 = (selected_targets3(2:end,:) + selected_targets3(1:end-1,:))/2;
forbidArea3(end+1,:) = forbidArea3(1,:);
forbidArea3_3 = zeros(size(forbidArea3,1),3);
forbidArea3_3(:,1:2) = forbidArea3(:,[2,1]);
p_forbidArea3 = lla2ecef(forbidArea3_3, 'WGS84');
p_forbidArea3(:,3) = [];

p_forbidArea = cell(1,3);
p_forbidArea{1,1} = p_forbidArea1';
p_forbidArea{1,2} = p_forbidArea2';
p_forbidArea{1,3} = p_forbidArea3';

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

p_all = [p_agent ; p_targets ; p_task];

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

for i = 1 : length(p_forbidArea)
    p_forbidArea{1,i}(1,:) = p_forbidArea{1,i}(1,:) - deviation_x;
    p_forbidArea{1,i}(2,:) = p_forbidArea{1,i}(2,:) - deviation_y;
end

targets_angle = [60,210,105,135,30,240,120,165,15,45,75,330,60,255,180,225,285,120,75,150,300,90,270,285,240,180,255,60,240,150,300,90,270,285,240]';

targets_restCheck = [3,4,4,5,4,3,3,4,5,4,3,4,5,4,3,5,4,5,4,4,5,4,5,4,5,3,4,5,4,4,5,4,5,4,5]';

%% 计算全图覆盖航路点
na = 8;
x = p_task(1,:)';
y = p_task(2,:)';
[lmin,lmax,V,laneDist] = findStrips(x,y,0,400,400);
lmin = sortrows(lmin,1);
lmax = sortrows(lmax,1);
pos_waypoints = [lmin;lmax];
lineNo = size(lmin,1);
EachLine_agent = floor(lineNo/na)*ones(na,1);
remainedLineNo = mod(lineNo,na);
tmp = zeros(na,1);
tmp(1:remainedLineNo)=1;
EachLine_agent = EachLine_agent + tmp;
Waypoints_agent = cell(na,1);
lmin_store = lmin;
lmax_store = lmax;
iterRounds = 3000;
%剔除禁航区内的航路点
for i=1:na
    Waypoints_agent{i} = [lmax_store(1:EachLine_agent(i),:) lmin_store(1:EachLine_agent(i),:)];
    lmax_store(1:EachLine_agent(i),:) = [];
    lmin_store(1:EachLine_agent(i),:) = [];
end
[X,xmax] = fly2waypoints(na, pos_a, Waypoints_agent,iterRounds);

traj_points = cell(na,1)';
for i=1:na
    traj = X{i}(1:2,:)';
    for j = 1 : length(traj)
        for k = 1: length(p_forbidArea)
            if inforbid(p_forbidArea{1,k},traj(j,:))
                traj(j,:) = 0;
            end
        end
    end
    traj(find(traj(:,1)==0),:)=[];
    traj_points{i} = traj(1:20:end,:)';
end

% hold on;
% colors = lines(na);
% % plotMapAllocation(X, na, colors, 'Planned Path');
% plotMapAllocation(traj_points, na, colors, 'Planned Path');
% plot(p_forbidArea(1,:),p_forbidArea(2,:),'r','DisplayName', 'ForbidArea','LineWidth',2);

%% 根据航路点初始化智能体
agents = ini_Agent_test(pos_a , traj_points);
dt = 3;
obs = [];
for i = 1 : length(pos_t)
    obs = [obs ; addObs(num2str(i), pos_t(i,:), [0 0])];
end
targets_searched = [];
maxIterations = 1500;
counter = 0;
colors = lines(na);

%% Stage 1核心代码
flag = 0;
while flag == 0
    % maxDistFromGoal = 0;
    %%%%%% part 1：求智能体的避障参数%%%%%%%%%%%%
    for i = 1:length(agents)
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
        if ~isempty(agents(i).goal)
            agents(i).newControl = getControls(agents(i), obstacles, dt, p_forbidArea, p_task);
        else
            agents(i).newControl = [0,0];
        end
    end
    
    %%%%%%%%%%%% part 2：更新航路点%%%%%%%%%%%%%%%%%%%%
    for i = 1:length(agents)
        agents(i).path = [agents(i).path; agents(i).position];
        agents(i).position = futurePosition(agents(i), dt);
        pos_agent(i,:) = agents(i).position;
        agents(i).velocity = agents(i).newControl;
        %         maxDistFromGoal = max(maxDistFromGoal, sum((agents(i).position - agents(i).goal).^2));
        if ~isempty(traj_points{i})
            if  norm(agents(i).position - agents(i).goal) <= 60 %%这个阈值需要综合考虑dt和Vmax
                traj_points{i}(:,1) = [];
                agents = updateAgent_test(agents , traj_points);
            end
        else
            agents(i).goal = [];
        end
    end
    
    %%%%%%%%%%% part 3：绘制结果%%%%%%%%%%%%%%%%%%
    clf('reset')
    hold on
    plot(p_task(1,:),p_task(2,:),'k','DisplayName', 'Boundary','LineWidth',2);
    for i = 1 : length(p_forbidArea)
        plot(p_forbidArea{1,i}(1,:),p_forbidArea{1,i}(2,:),'r','DisplayName', 'ForbidArea','LineWidth',2);
    end
    
    %     plot(pos_waypoint(:,1),pos_waypoint(:,2),'ko','LineWidth',3,'MarkerSize',15, 'DisplayName', 'Waypoints')
    
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
    flag = 1;
    for i=1:length(traj_points)
        flag = isempty(traj_points{i}) & flag;
    end
end