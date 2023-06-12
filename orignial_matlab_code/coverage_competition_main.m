addpath('GreedyCoalitionAuctionAlgorithm/');
close all; clear all;

%% Stage 1: 区域覆盖及静目标搜索

%%%%%%%%%%%%%%%%%%%%%%%%%%%%初始参数设置%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
uniform_agents = 0;
uniform_tasks = 1;%%每个task外围盘旋的半径是否一致
plot_range = 1;

na = 8;
nt = 34;
n_rounds = 2000;

Lt = 1;
nt_loiter = ceil(0*nt);  %%nt的系数决定了绕圈agent的数量
task_type = zeros(nt,1);%%哪几个Agent绕圈、哪几个Agent直线接近
task_type(1:nt_loiter) = 1;
task_type1 = ones(nt,1);
lambda = 1;

map_width = 7000;
comm_distance = 0.01 * map_width;

simu_time = 10;
time_step = 0.05;
time_start = 0;

vesPostion = [121.64994868, 38.82776073;
    121.64994868, 38.82775873;
    121.64994868, 38.82775673;
    121.64994868, 38.82775473;
    121.64994868, 38.82775273;
    121.64994868, 38.82775073;
    121.64994868, 38.82774873;
    121.64994868, 38.82774673;];
vesPostion_3 = zeros(size(vesPostion,1),3);
vesPostion_3(:,1:2) = vesPostion;
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
    121.71204920406547, 38.80605631718123];
targets_pos_value_3 = zeros(size(targets_pos_value,1),3);
targets_pos_value_3(:,1:2) = targets_pos_value;
p_targets = lla2ecef(targets_pos_value_3, 'WGS84');
p_targets(:,3) = [];

taskArea = [121.66473222 , 38.85573388;
    121.71089122 , 38.85573388;
    121.71940282 , 38.82943606;
    121.71940282 , 38.79479375;
    121.70958175 , 38.76925451;
    121.66505958 , 38.76925451;
    121.65572957 , 38.79441445;
    121.65572957 , 38.82791887;
    121.66473222 , 38.85573388;];
taskArea_3 = zeros(size(taskArea,1),3);
taskArea_3(:,1:2) = taskArea;
p_task = lla2ecef(taskArea_3, 'WGS84');
p_task(:,3) = [];

p_all = [p_agent ; p_targets ; p_task];

p_agent(:,1) = p_agent(:,1) - min(p_all(:,1));
p_agent(:,2) = p_agent(:,2) - min(p_all(:,2));
pos_a = p_agent;

p_targets(:,1) = p_targets(:,1) - min(p_all(:,1));
p_targets(:,2) = p_targets(:,2) - min(p_all(:,2));
pos_t = p_targets;


p_task = p_task';
p_task(1,:) = p_task(1,:) - min(p_all(:,1));
p_task(2,:) = p_task(2,:) - min(p_all(:,2));

targets_angle = [60 ; 210 ; 105 ; 135 ; 30 ; 240 ; 120 ; 165 ; 15 ; 45 ; 75 ; 330 ; 60 ; 255 ; 180 ; 225 ; 285 ; 120 ; 75 ; 150 ; 300 ; 90 ; 270 ; 285 ; 240];

targets_restCheck = [3 ; 4 ; 4 ; 5 ; 4 ; 3 ; 3 ; 4 ; 5 ; 4 ; 3 ; 4 ; 5 ; 4 ; 3 ; 5 ; 4 ; 5 ; 4 ; 4 ; 5 ; 4 ; 5 ; 4 ; 5];

x = p_task(1,:)';
y = p_task(2,:)';
[lmin,lmax,V,laneDist] = findStrips(x,y,0,300,300);
lmin = sortrows(lmin,1);
lmax = sortrows(lmax,1);
pos_waypoints = [lmin;lmax];

tf_t =  simu_time * (1.95 + 0.05 * rand(nt,1)); %% tf_t会影响任务分配结果
tloiter_t =  simu_time * (0.2 + 0.05 * rand(nt,1));
tloiter_t(task_type == 0) = 0;

pos_t_initial = pos_waypoints;

kdrag = 3 / simu_time;

max_speed = 1;
if uniform_agents
    v_a = zeros(na,2);
else
    v_a = (2 * rand(na,2) - 1) * max_speed;
end

max_speed_task = 0.1;
if uniform_tasks
    v_t = zeros(nt,2);
else
    v_t = (2 * rand(nt,2) - 1) * max_speed_task;
end

R = 300;%%task外围盘旋的半径
if uniform_tasks
    radius_t = R * ones(nt,1);
else
    radius_t = (0.2 * rand(nt,1) + 1) * R;
end

R2 = 50;
radius_t_2 = R2 * ones(nt,1);

% Reward after task completion
r_nom = 0.2;
if uniform_tasks
    r_bar = r_nom * ones(nt,1);
else
    r_bar = r_nom * rand(nt,1);
end
r_bar(task_type == 1) = 5 * r_bar(task_type == 1);

% Probability that agent i successfully completes task j
if uniform_agents
    prob_a_t = 0.7 * ones(na,nt);
else
    prob_a_t = rand(na,nt);
end

Tasks.r_bar = r_bar;
Tasks.prob_a_t = prob_a_t;
Tasks.task_type = task_type;

Agents.N = na;
Agents.Lt = Lt * ones(1,na);
Agents.v_a = v_a;
Agents.previous_task = zeros(na,1);
Agents.previous_winnerBids = zeros(na,1);
Agents.rin_task = zeros(na,2);
Agents.vin_task = zeros(na,2);
Agents.kdrag = kdrag;

% Fully connected graph
G = ~eye(Agents.N);

figure; hold on;
colors = lines(na);

n_rounds_loop = n_rounds;
simu_time_loop = simu_time;
time_start_loop = time_start;
tf_t_loop = tf_t;
pos_a_loop = pos_a;
v_a_loop = v_a;

completed_tasks_round = [];
completed_tasks = [];

i_round=1;

p_GCAA = mat2cell([1   3	 5	 7	 9	 11	 13	 15],1,[1 1 1 1 1 1 1 1]);
ind_completed = [];
targets_searched = [];
dynamic_flag = zeros(na,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Stage1 核心代码 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while sum(cell2mat(p_GCAA))~=0 %%程序运行直到静目标全部找到为止
    break;
    mat_p_GCAA = cell2mat(p_GCAA);
    mat_p_GCAA(find(mat_p_GCAA == 0))=[] ;
    pos_waypoint = pos_waypoints(mat_p_GCAA,:);

    clf; hold on;
    xlim([0 map_width]);
    ylim([0 map_width]);
    xlabel('x [m]');
    ylabel('y [m]');
    title('Stage 1: Area Coverage and Targets Searched:',num2str(size(targets_searched,1)),'FontSize',20);
    for i = 1:na
        plot(pos_a_loop(i,1), pos_a_loop(i,2), '*', 'Color', colors(i,:), 'MarkerSize', 10, 'DisplayName', 'Agents');
        for j = 1:length(p_targets)
            if norm(pos_a_loop(i,:)-p_targets(j,:))<=300
                plot(p_targets(j,1), p_targets(j,2),'rs', 'MarkerSize', 10, 'DisplayName', 'Targets', 'MarkerFaceColor',[1 .6 .6])
                if isempty(find(targets_searched==p_targets(j,1)))
                    targets_searched = [targets_searched;p_targets(j,:)];
                end
            end
        end
    end

    if ~isempty(targets_searched)
        plot(targets_searched(:,1), targets_searched(:,2),'rs', 'MarkerSize', 10, 'DisplayName', 'Targets', 'MarkerFaceColor',[1 .6 .6])
    end
 
    for w = 1:length(pos_waypoint)
        if isempty(find(completed_tasks == w))
            plot(pos_waypoint(:,1),pos_waypoint(:,2),'ko','LineWidth',3,'MarkerSize',15, 'DisplayName', 'Waypoints')
        end
    end

    plot(p_task(1,:),p_task(2,:),'k','DisplayName', 'Boundary','LineWidth',2);

    Agents.Pos = pos_a_loop;
    Agents.v_a = v_a_loop;

    Tasks.Pos = pos_waypoints;
    Tasks.Speed = v_t;
    Tasks.N = nt;
    Tasks.tf = tf_t_loop;
    Tasks.lambda = lambda;
    Tasks.task_type = task_type;
    Tasks.tloiter = tloiter_t;
    Tasks.radius = radius_t;
    Tasks.angle = targets_angle;
    Tasks.restCheck = targets_restCheck;

    [X, completed_tasks_round] = OptimalControlSolution_stage1(pos_a_loop, v_a_loop, pos_waypoints, v_t, radius_t, p_GCAA, Agents, tf_t_loop, tloiter_t, time_step, n_rounds_loop, na, kdrag);

    plotMapAllocation_stage1(X, n_rounds_loop, na, colors, 'Planned Path');

    legend(legendUnq(gca));

    drawnow;

    % Update position and velocity of each agent
    pos_a_loop = X(1:2,:,2)';
    v_a_loop   = X(3:4,:,2)';

    simu_time_loop = simu_time_loop - time_step;
    time_start_loop = time_start_loop + time_step;
    n_rounds_loop = n_rounds_loop - 1;
    tf_t_loop = tf_t_loop - time_step;%tf_t_loop在不断减少

    if ~isempty(completed_tasks_round)
        for k = 1 : size(completed_tasks_round,2)
            ind_completed = find(cell2mat(p_GCAA) == completed_tasks_round(k));
            if dynamic_flag(ind_completed)==0
                p_GCAA{1,ind_completed} = p_GCAA{1,ind_completed}+17;
                dynamic_flag(ind_completed) = dynamic_flag(ind_completed)+1;
            elseif dynamic_flag(ind_completed)==1
                p_GCAA{1,ind_completed} = p_GCAA{1,ind_completed}+1;
                dynamic_flag(ind_completed) = dynamic_flag(ind_completed)+1;
            elseif dynamic_flag(ind_completed)==2
                p_GCAA{1,ind_completed} = p_GCAA{1,ind_completed}-17;
                dynamic_flag(ind_completed) = dynamic_flag(ind_completed)+1;
            elseif dynamic_flag(ind_completed)==3
                p_GCAA{1,ind_completed} = 0;
            end
        end

        Agents.N = na;
        Agents.Lt = Lt * ones(1,na);
        Agents.previous_task = zeros(na,1);
        Agents.previous_winnerBids = zeros(na,1);
        Agents.rin_task = zeros(na,2);
        Agents.vin_task = zeros(na,2);
        Agents.kdrag = kdrag;

        n_rounds_loop = n_rounds;
    end


    i_round = i_round+1;

end

clf; hold on;
xlim([0 map_width]);
ylim([0 map_width]);
xlabel('x [m]');
ylabel('y [m]');
for i = 1:na
    plot(pos_a_loop(i,1), pos_a_loop(i,2), '*', 'Color', colors(i,:), 'MarkerSize', 10, 'DisplayName', 'Agents');
end
plot(p_task(1,:),p_task(2,:),'k','DisplayName', 'Boundary','LineWidth',2);
title('Stage 2: Static Target Exploration','FontSize',20);
legend(legendUnq(gca));
drawnow;

%% Stage 2: 静目标探查

%%%%%%%%%%%%%%%%%%%%% 初始参数设置 %%%%%%%%%%%%%%%%%%%%%%%%%%
na = 8;
nt = 25;
nt_ini = nt;
n_rounds = 5000;

targets_angle = [60 ; 210 ; 105 ; 135 ; 30 ; 240 ; 120 ; 165 ; 15 ; 45 ; 75 ; 330 ; 60 ; 255 ; 180 ; 225 ; 285 ; 120 ; 75 ; 150 ; 300 ; 90 ; 270 ; 285 ; 240];
targets_restCheck = [3 ; 4 ; 4 ; 5 ; 4 ; 3 ; 3 ; 4 ; 5 ; 4 ; 3 ; 4 ; 5 ; 4 ; 3 ; 5 ; 4 ; 5 ; 4 ; 4 ; 5 ; 4 ; 5 ; 4 ; 5];
changeTargetsNo = 5;
% changeTargets = zeros(4,changeTargetsNo);
changeTargets = [5 10 15 20 25;45 60 180 135 120;4 4 3 4 5;2 1 2 1 1]; % column: id, angle, restCheck, changeCheck

targets_restCheck(changeTargets(1,:)) = changeTargets(4,:);

pos_t_initial = pos_t;
pos_t_initial(:,end+1)=(1:nt)';

R = 300; %%%%%%% task外围盘旋的半径
R2 = 50; %%%%%%% 威胁区半径
radius_t = R * ones(nt,1);
radius_t_2 = R2 * ones(nt,1);

Lt = 1;
task_type = zeros(nt,1);%%哪几个Agent绕圈、哪几个Agent直线接近
task_type1 = ones(nt,1);
lambda = 1;

map_width = 7000;
comm_distance = 0.01 * map_width;

simu_time = 10;
% time_step = simu_time / n_rounds;
time_step = 0.05;
time_start = 0;
tf_t =  simu_time * (1.95 + 0.05 * rand(nt,1)); %% tf_t会影响任务分配结果
tloiter_t =  simu_time * (0.2 + 0.05 * rand(nt,1));
tloiter_t(task_type == 0) = 0;

uniform_agents = 0;
uniform_tasks = 1;%%每个task外围盘旋的半径是否一致
plot_range = 1;

kdrag = 3 / simu_time;

max_speed = 1;
if uniform_agents
    v_a = zeros(na,2);
else
    v_a = (2 * rand(na,2) - 1) * max_speed;
end

max_speed_task = 0.1;
if uniform_tasks
    v_t = zeros(nt,2);
else
    v_t = (2 * rand(nt,2) - 1) * max_speed_task;
end

r_nom = 0.2;
if uniform_tasks
    r_bar = r_nom * ones(nt,1);
else
    r_bar = r_nom * rand(nt,1);
end
r_bar(task_type == 1) = 5 * r_bar(task_type == 1);

% Probability that agent i successfully completes task j
if uniform_agents
    prob_a_t = 0.7 * ones(na,nt);
else
    prob_a_t = rand(na,nt);
end

Tasks.r_bar = r_bar;
Tasks.prob_a_t = prob_a_t;
Tasks.task_type = task_type;


Agents.N = na;
Agents.Lt = Lt * ones(1,na);
Agents.v_a = v_a;
Agents.previous_task = zeros(na,1);
Agents.previous_winnerBids = zeros(na,1);
Agents.rin_task = zeros(na,2);
Agents.vin_task = zeros(na,2);
Agents.kdrag = kdrag;

costs = zeros(na, nt);
utility = zeros(na, nt);
rewards = zeros(na, nt);

% Fully connected graph
G = ~eye(Agents.N);

hold on;
colors = lines(na);

n_rounds_loop = n_rounds;
simu_time_loop = simu_time;
time_start_loop = time_start;
tf_t_loop = tf_t;
v_a_loop = v_a;

U_next_tot = zeros(n_rounds,1);
U_tot = zeros(n_rounds,1);
U_completed_tot = 0;

completed_tasks_round = [];
completed_tasks = [];
rt_completed = 0;

X_full_simu{n_rounds} = 0;
p_GCAA_full_simu{n_rounds} = 0;
S_GCAA_ALL_full_simu = zeros(n_rounds,nt);
rt_full_simu = zeros(n_rounds,nt);
J = zeros(n_rounds,na); % Cost for each agent from start to current step
J_to_completion_target = zeros(n_rounds,na); % Estimated cost for each agent from step to end


flag = 1;
flag_changeTargets = ones(nt,1);
flag_changeTargets(changeTargets(1,:)) = flag_changeTargets(changeTargets(1,:))+1;

Agents.Pos = pos_a_loop;
Agents.v_a = v_a_loop;

pos_t(:,end+1)=(1:nt)';
Tasks.Pos = pos_t;
Tasks.Speed = v_t;
Tasks.N = nt;
Tasks.tf = tf_t_loop;
Tasks.lambda = lambda;
Tasks.task_type = task_type;
Tasks.task_type1 = task_type1;
Tasks.tloiter = tloiter_t;
Tasks.radius = radius_t;
Tasks.radius_t_2 = radius_t_2;
Tasks.angle = targets_angle;
Tasks.restCheck = targets_restCheck;
Tasks.completed = zeros(nt,1);
Tasks.restCheck = targets_restCheck;
Tasks.flag_changeTargets = flag_changeTargets;



completed_agents = [];
assigned_tasks = [];
Xfinal = cell(na,1);
Tasks_initial = Tasks;
completed_tasks_Store = [];

%%%%%%%%%%%%%%%%%%%%%%%%%% Stage 2核心代码 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i_round=1:n_rounds
    completed_tasks_round=[];
    %%%%%%%%%%%%%%%%% Initial Plot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    clf; hold on;
    xlim([0 map_width]);
    ylim([0 map_width]);
    xlabel('x [m]');
    ylabel('y [m]');
    title('Stage 2: Static Target Exploration','FontSize',20);
    for i = 1:na
        plot(pos_a_loop(i,1), pos_a_loop(i,2), '*', 'Color', colors(i,:), 'MarkerSize', 10, 'DisplayName', 'Agents');
    end
    plot(Tasks_initial.Pos(:,1), Tasks_initial.Pos(:,2),'rs', 'MarkerSize', 10, 'DisplayName', 'Targets', 'MarkerFaceColor',[1 .6 .6]);
    PlotTaskLoitering(Tasks_initial.Pos(:,1:2), Tasks_initial.radius, Tasks_initial.task_type1, 'b', 'Exploration Range');
    PlotTaskLoitering(Tasks_initial.Pos(:,1:2), Tasks_initial.radius_t_2, Tasks_initial.task_type1, 'r', 'Threaten Range');
    plot(p_task(1,:),p_task(2,:),'k','DisplayName', 'Boundary','LineWidth',2);
    for i=1:size(Tasks_initial.Pos,1)
        text(Tasks_initial.Pos(i,1)+100,Tasks_initial.Pos(i,2)+100,num2str(Tasks_initial.Pos(i,3)),'FontSize',15);
    end
    
    if flag && nt~=0
        [~, p_GCAA_tmp,taskInd, ~, ~, Agents] = GCAASolution_revised(Agents, G, Tasks); %% p_GCAA表示被分配任务的task
        assigned_tasks = cell2mat(p_GCAA_tmp);
        %         p_GCAA_tmp =  p_GCAA_tmp(~cellfun('isempty', p_GCAA_tmp));
        %         assigned_tasks = taskInd;
        [Xmed, ~, ~, ~] = OptimalControlSolution_stage2(Agents,Tasks,p_GCAA_tmp,time_step,n_rounds_loop,kdrag);
        if i_round==1
            Xfinal = Xmed;
            %             p_GCAA = p_GCAA_tmp;
            p_GCAA = num2cell(taskInd);
        else
            for j=1:size(Xmed,1)
                mm=1;
                Xfinal{completed_agents(j)} = Xmed{j};
                if isempty(p_GCAA_tmp{j})
                    p_GCAA{completed_agents(j)} = [];
                else
                    p_GCAA{completed_agents(j)} = taskInd(mm);    % p_GCAA_tmp{j};
                    mm = mm+1;
                end
            end
        end
        flag = 0;
        completed_agents = [];
        
        %%%%%%%%%%%%%%%%%%%% Update Changed Targets %%%%%%%%%%%%%%%%%%
        flag_changeTargets(taskInd) = flag_changeTargets(taskInd)-1;
        %         ind5 = find(changeTargets(1,:)==taskInd);
        %         if ~isempty(ind5)
        %             flag_changeTargets(ind5) = flag_changeTargets(ind5)-1;
        %         end
        
        %%%%%%%%%%%%%%%%%% Update Available Targets for GCAA %%%%%%%%%%%%%
        pos_t(assigned_tasks,:) = [];
        v_t(assigned_tasks,:) = [];
        nt = nt - size(assigned_tasks,2);
        tf_t(assigned_tasks,:) = [];
        tf_t_loop = tf_t;
        task_type(assigned_tasks,:) = [];
        task_type1(assigned_tasks,:) = [];
        tloiter_t(assigned_tasks,:) = [];
        radius_t(assigned_tasks,:) = [];
        radius_t_2(assigned_tasks,:) = [];
        Tasks.r_bar(assigned_tasks,:) = [];
        Tasks.prob_a_t(:,assigned_tasks) = [];
        Tasks.Pos = pos_t;
        Tasks.Speed = v_t;
        Tasks.N = nt;
        Tasks.tf = tf_t_loop;
        Tasks.lambda = lambda;
        Tasks.task_type = task_type;
        Tasks.tloiter = tloiter_t;
        Tasks.radius = radius_t;
        Tasks.angle(assigned_tasks) = [];
        Tasks.restCheck(assigned_tasks) = [];
        Tasks.flag_changeTargets(assigned_tasks) = [];
        
        assigned_tasks = [];
    end
    
    plotMapAllocation_stage2(Xfinal, na, colors, 'Planned Path');
    legend(legendUnq(gca));
    drawnow;
    
    % Update position and velocity of each agent
    
    for i=1:na
        if isempty(Xfinal{i})
            tmp = p_GCAA{i};
            %             completed_tasks_round = [completed_tasks_round tmp];
            if isempty(intersect(tmp,completed_tasks_Store)) && ~isempty(tmp) %%%%% not sure if ~isempty(tmp) is needed or not 
                completed_agents = [completed_agents i];
                completed_tasks_round = [completed_tasks_round tmp];
                flag = 1;
                if flag_changeTargets(tmp)==0
                    completed_tasks_Store = [completed_tasks_Store tmp];
                end
            end
        else
            pos_a_loop(i,:) = Xfinal{i}(1:2,1)';
            v_a_loop(i,:)   = Xfinal{i}(3:4,1)';
            Xfinal{i}(:,1)=[];
        end
    end
    
    n_rounds_loop = n_rounds_loop - 1;
    simu_time_loop = simu_time_loop - time_step;
    time_start_loop = time_start_loop + time_step;
    tf_t_loop = tf_t_loop - time_step;
    
    if ~isempty(completed_tasks_round)
        %%%%%%%%%%%%% for Plot Targets %%%%%%%%%%%%%%%%%%%%%
        tmp1 = Tasks_initial.Pos(:,3);
        [~,ind1]=ismember(completed_tasks_round,tmp1);
        for j=1:length(ind1)
            if ind1(j)~=0 && flag_changeTargets(completed_tasks_round(j))==0
                pos_t_tmp = Tasks_initial.Pos;
                radius_t_tmp = Tasks_initial.radius;
                radius_t_2_tmp = Tasks_initial.radius_t_2;
                task_type1_tmp = Tasks_initial.task_type1;
                pos_t_tmp(ind1(j),:) = [];
                radius_t_tmp(ind1(j),:) = [];
                radius_t_2_tmp(ind1(j),:) = [];
                task_type1_tmp(ind1(j),:) = [];
                Tasks_initial.Pos = pos_t_tmp;
                Tasks_initial.radius = radius_t_tmp;
                Tasks_initial.radius_t_2 = radius_t_2_tmp;
                Tasks_initial.task_type1 = task_type1_tmp;
                %%%%%%%%%%%%%%%%%%% Update change targets %%%%%%%%%%%%%%%%%%%
            elseif flag_changeTargets(completed_tasks_round(j))~=0
                ind2 = completed_tasks_round(j);
                ind3 = find(changeTargets(1,:)==ind2);
                pos_t(end+1,:) = pos_t_initial(ind2,:);
                v_t(end+1,:) = [0 0];
                nt = nt + 1;
                tf_t(end+1,:) = simu_time*(1.95+0.05*rand);
                tf_t_loop = tf_t;
                task_type(end+1,:) = 0;
                task_type1(end+1,:) = 1;
                tloiter_t(end+1,:) = 0;
                radius_t(end+1,:) = R;
                radius_t_2(end+1,:) = R2;
                Tasks.r_bar(end+1,:) = r_nom;
                Tasks.prob_a_t(:,end+1) = rand(size(Tasks.prob_a_t,1),1);
                Tasks.Pos = pos_t;
                Tasks.Speed = v_t;
                Tasks.N = nt;
                Tasks.tf = tf_t_loop;
                Tasks.lambda = lambda;
                Tasks.task_type = task_type;
                Tasks.tloiter = tloiter_t;
                Tasks.radius = radius_t;
                Tasks.radius_t_2 = radius_t_2;
                Tasks.angle(end+1) = changeTargets(2,ind3);
                Tasks.restCheck(end+1) = changeTargets(3,ind3);
                Tasks.flag_changeTargets(end+1) = flag_changeTargets(ind2);
                %             flag_changeTargets(completed_tasks_round(j)) = flag_changeTargets(completed_tasks_round(j))-1;
            end
        end
        %%%%%%%%%%%%%%%%%%% for Update Agents %%%%%%%%%%%%%%%%%%%%%
        na_new = length(completed_agents);
        v_a_loop = zeros(na_new,2);
        Agents.N = na_new;
        Agents.Pos = pos_a_loop(completed_agents,:);
        Agents.Lt = Lt * ones(1,na_new);
        Agents.previous_task = zeros(na_new,1);
        Agents.previous_winnerBids = zeros(na_new,1);
        Agents.rin_task = zeros(na_new,2);
        Agents.vin_task = zeros(na_new,2);
        Agents.kdrag = kdrag;
        Agents.v_a=v_a_loop;
        
        Tasks.prob_a_t = rand(na_new,nt);
        
        n_rounds_loop = n_rounds;
        
    end
    
    if length(completed_tasks_Store)==nt_ini
        break;
    end
end
clf; hold on;
xlim([0 map_width]);
ylim([0 map_width]);
xlabel('x [m]');
ylabel('y [m]');
title('Stage 2: Static Target Exploration','FontSize',20);
for i = 1:na
    plot(pos_a_loop(i,1), pos_a_loop(i,2), '*', 'Color', colors(i,:), 'MarkerSize', 10, 'DisplayName', 'Agents');
end
plot(pos_t(:,1), pos_t(:,2),'rs', 'MarkerSize', 8, 'DisplayName', 'Targets', 'MarkerFaceColor',[1 .6 .6]);
plot(p_task(1,:),p_task(2,:),'k','DisplayName', 'Boundary','LineWidth',2);
legend(legendUnq(gca));
drawnow;
%%


function pos_t_new = RemoveCompletedTasks(pos_t, ind)

pos_t_new = zeros(size(pos_t,1)-length(ind),2);
k = 1;
for t = 1:size(pos_t,1)
    if ~sum(ind == t)
        pos_t_new(k,:) = pos_t(t,:);
        k = k + 1;
    end
end

end

function [p, pos_a, ind_completed_tasks, nt, Agents] = UpdatePath(p, pos_a, pos_t, time_step, Agents, nt)
ind_completed_tasks = [];

for i = 1:size(pos_a,1)
    if ~isempty(p{i})
        d_a_t = pos_t(p{i}(1),:) - pos_a(i,:);
        if (norm(d_a_t) < time_step * Agents.Speed(i))
            pos_a(i,:) = pos_t(p{i}(1),:);
            nt = nt - 1;
            Agents.Lt(i) = Agents.Lt(i) - 1;
            ind_completed_tasks = [ind_completed_tasks p{i}(1)];
            
            p{i} = p{i}(2:end);
            %                 if ~isempty(p{i})
            %                     time_step_remaining = time_step - norm(d_a_t) / Agents.Speed(i);
            %                     d_a_next_t = pos_t(p{i}(1),:) - pos_a(i,:);
            %                     pos_a(i,:) = pos_a(i,:) + d_a_next_t / norm(d_a_next_t) * time_step_remaining * Agents.Speed(i);
            %                 end
        else
            pos_a(i,:) = pos_a(i,:) + d_a_t / norm(d_a_t) * time_step * Agents.Speed(i);
        end
    end
end
end

