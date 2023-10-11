clc
clear all
close all

%% 加载GCAA代码工具箱
load('p_task.mat');
load('pos_agent.mat');
load('pos_t.mat');
load('targets_angle.mat');
load('targets_restCheck.mat');
load('p_forbidArea.mat');

%% Stage 2: 静目标探查

%%%%%%%%%%%%%%%%%%%%% 初始参数设置 %%%%%%%%%%%%%%%%%%%%%%%%%%
na = 8;
nt = 35;
nt_ini = nt;
n_rounds = 5000;

changeTargets = [5 10 15 20 25 30 35;150 225 180 300 120 345 330;3 2 3 5 4 1 4;2 1 2 2 2 1 3]; % column: id, angle, restCheck, changeCheck

targets_restCheck(changeTargets(1,:)) = changeTargets(4,:);

pos_t_initial = pos_t;
pos_t_initial(:,end+1)=(1:nt)';

R = 150; %%%%%%% task外围盘旋的半径
R2 = 50; %%%%%%% 威胁区半径
radius_t = R * ones(nt,1);
radius_t_2 = R2 * ones(nt,1);

Lt = 1;
task_type = zeros(nt,1);%%哪几个Agent绕圈、哪几个Agent直线接近
task_type1 = ones(nt,1);
lambda = 1;

map_width = 7250;
comm_distance = 0.01 * map_width;

simu_time = 10;
% time_step = simu_time / n_rounds;
time_step = 1;
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
    %     prob_a_t = rand(na,nt);
    prob_a_t = zeros(na,nt);
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
% flag_changeTargets = ones(nt,1);
% flag_changeTargets(changeTargets(1,:)) = flag_changeTargets(changeTargets(1,:))+1;

Agents.Pos = pos_agent;
pos_a_loop = pos_agent;
Agents.v_a = v_a_loop;

% pos_t(:,end+1)=(1:nt)';
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
% Tasks.flag_changeTargets = flag_changeTargets;

completed_agents = 1:na;
assigned_tasks = [];
Xfinal = cell(na,1);
Tasks_initial = Tasks;
Tasks1 = Tasks;
restCheck = targets_restCheck;
completed_tasks_Store = [];
M = -9999;
timeLapse = zeros(na,nt);

agents = ini_Agent_test_stage2(pos_agent , pos_a_loop);
dt = 3;
obs = [];
pos_t_coor = pos_t(:,1:2);
for i = 1 : length(pos_t_coor)
    obs = [obs ; addObs(num2str(i), pos_t_coor(i,:), [0 0])];
end
maxIterations = 1500;
counter = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%% Stage 2 核心代码 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i_round=1:n_rounds
    completed_tasks_round=[];
    %%%%%%%%%%%%%%%%% Initial Plot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    clf; hold on;
    xlim([0 map_width]);
    ylim([0 map_width]);
    xlabel('x [m]');
    ylabel('y [m]');
    title('Stage 2: Static Target Exploration','FontSize',20);

    maxDistFromGoal = 0;
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
            end
        end

        agents(i).newControl = getControls(agents(i), obstacles, dt, p_forbidArea);
    end

    %%%%%%%%%%%% part 2：更新航路点%%%%%%%%%%%%%%%%%%%%
    for i = 1:length(agents)
        agents(i).path = [agents(i).path; agents(i).position];
        agents(i).position = futurePosition(agents(i), dt);
        pos_agent(i,:) = agents(i).position;
        agents(i).velocity = agents(i).newControl;
        maxDistFromGoal = max(maxDistFromGoal, sum((agents(i).position - agents(i).goal).^2));
        if  norm(agents(i).position - agents(i).goal) <= 60 %%这个阈值需要综合考虑dt和Vmax
            agents = updateAgent_test_stage2(agents , pos_a_loop);
        end
    end

%     for i = 1:na
%         plot(pos_a_loop(i,1), pos_a_loop(i,2), '*', 'Color', colors(i,:), 'MarkerSize', 10, 'DisplayName', 'Agents');
%     end
    for i = 1:length(agents)
        plot(agents(i).position(1), agents(i).position(2), '*', 'Color', colors(i,:), 'MarkerSize', 10, 'DisplayName', 'Agents');
        PlotTaskLoitering_stage1(agents(i).position, agents(i).radius, 1, 'g', 'Agent Threaten Range');
        text(agents(i).position(1), agents(i).position(2), agents(i).name);
    end

    plot(Tasks_initial.Pos(:,1), Tasks_initial.Pos(:,2),'rs', 'MarkerSize', 10, 'DisplayName', 'Targets', 'MarkerFaceColor',[1 .6 .6]);
    PlotTaskLoitering(Tasks_initial.Pos(:,1:2), Tasks_initial.radius, Tasks_initial.task_type1, 'b', 'Exploration Range');
    PlotTaskLoitering(Tasks_initial.Pos(:,1:2), Tasks_initial.radius_t_2, Tasks_initial.task_type1, 'r', 'Threaten Range');
    plot(p_task(1,:),p_task(2,:),'k','DisplayName', 'Boundary','LineWidth',2);
    plot(p_forbidArea(1,:),p_forbidArea(2,:),'r','DisplayName', 'ForbidArea','LineWidth',2);
    for i=1:size(Tasks_initial.Pos,1)
        str = strcat(num2str(Tasks_initial.Pos(i,3)),'(',num2str(Tasks_initial.restCheck(i)),')');
        text(Tasks_initial.Pos(i,1)+100,Tasks_initial.Pos(i,2)+100,str,'FontSize',15);
    end
    
    
    [row,col] = find(timeLapse>0);
    for k=1:length(row)
        if i_round-timeLapse(row(k),col(k))>=90
            Tasks.prob_a_t(row(k),col(k)) = 0;
            Tasks1.prob_a_t(row(k),col(k)) = 0;
            timeLapse(row(k),col(k))=0;
        end
    end
    
    
    if flag && nt~=0 && ~isempty(Tasks.Pos)
        [p_alloc, taskInd]=TaskAllocation(Agents, Tasks,completed_agents);
        %         [~, p_GCAA_tmp,taskInd, ~, ~, Agents] = GCAASolution_revised(Agents, G, Tasks); %% p_GCAA_tmp:当前被分配的任务序号;taskInd:在所有任务中的序号
        %         assigned_tasks = cell2mat(p_GCAA_tmp);
        p_GCAA_tmp = num2cell(p_alloc);
        p_GCAA_tmp(p_alloc == 0) = {};
        %         p_GCAA_tmp =  p_GCAA_tmp(~cellfun('isempty', p_GCAA_tmp));
        %         assigned_tasks = taskInd;
        [Xmed, ~, ~, ~] = OptimalControl_hcy0811(Agents,Tasks,p_GCAA_tmp,time_step,n_rounds_loop,kdrag);
        if i_round==1
            Xfinal = Xmed;
            %             p_GCAA = p_GCAA_tmp;
            p_GCAA = num2cell(taskInd);
        else
            for j=1:size(Xmed,1)
                %                 mm=1;
                Xfinal{completed_agents(j)} = Xmed{j};
                if isempty(p_GCAA_tmp{j})
                    p_GCAA{completed_agents(j)} = [];
                else
                    p_GCAA{completed_agents(j)} = taskInd(j);    % p_GCAA_tmp{j};
                    %                     mm = mm+1;
                end
            end
        end
        flag = 0;
%         Xfinal_Store = Xfinal;
        
        %%%%%%%%%%%%%%%%%% Update Available Targets that have rest checks %%%%%%%%%%%%%
        taskInd(taskInd==0)=[];
        restCheck(taskInd) = restCheck(taskInd)-1;
        [~,indtmp] = ismember(taskInd,Tasks1.Pos(:,3));
        indtmp(indtmp==0)=[];
        Tasks1.restCheck(indtmp) = Tasks1.restCheck(indtmp)-1;
        ind0 = find(Tasks1.restCheck==0);
        
        if ~isempty(ind0)
            nt = nt - length(ind0);
            Tasks1.r_bar(ind0,:) = [];
            %             Tasks.prob_a_t(:,ind0) = [];
            Tasks1.Pos(ind0,:) = [];
            Tasks1.Speed(ind0,:) = [];
            Tasks1.N = size(Tasks1.Pos,1);
            Tasks1.tf(ind0,:) = [];
            Tasks1.lambda = lambda;
            Tasks1.task_type(ind0,:) = [];
            Tasks1.task_type1(ind0,:) = [];
            Tasks1.tloiter(ind0,:) = [];
            Tasks1.radius(ind0,:) = [];
            Tasks1.radius_t_2(ind0,:) = [];
            Tasks1.angle(ind0) = [];
            Tasks1.restCheck(ind0) = [];
            %             Tasks1.flag_changeTargets(ind0) = [];
        end
        
        %%%%%%%%%%%%%%%%%% Update Available Targets for GCAA %%%%%%%%%%%%%
        Tasks = Tasks1;
        assigned_tasks = [];
        
    end
    
    completed_agents = [];
    
%     plotMapAllocation_stage2(Xfinal, na, colors, 'Planned Path');
    legend(legendUnq(gca));
    drawnow;
    
    
    % Update position and velocity of each agent
    for i=1:na
        if isempty(Xfinal{i}) && ~isempty(Xfinal_Store{i})
            tmp = p_GCAA{i};
            Xfinal_Store{i} = Xfinal{i};
            %             completed_tasks_round = [completed_tasks_round tmp];
            if isempty(intersect(tmp,completed_tasks_Store))  && ~isempty(tmp)  %%%%% not sure if ~isempty(tmp) is needed or not
                completed_agents = [completed_agents i];
                completed_tasks_round = [completed_tasks_round tmp];
                flag = 1;
                %                 [~,indx0] = ismember(tmp,Tasks.Pos(:,3));
                %                 indx0(indx0==0)=[];
                Tasks.prob_a_t(i,tmp) = M;
                Tasks1.prob_a_t(i,tmp) = M;
                timeLapse(i,tmp) = i_round;
                [~,indx] = ismember(tmp,Tasks_initial.Pos(:,3));
                indx(indx==0)=[];
                if ~isempty(indx)
                    Tasks_initial.restCheck(indx) = Tasks_initial.restCheck(indx)-1;
                end
                if restCheck(tmp)==0 && Tasks_initial.restCheck(indx)==0
                    completed_tasks_Store = [completed_tasks_Store tmp];                   
                    
                    Tasks_initial.Pos(indx,:) = [];
                    Tasks_initial.radius(indx,:) = [];
                    Tasks_initial.radius_t_2(indx,:) = [];
                    Tasks_initial.task_type1(indx,:) = [];
                    Tasks_initial.restCheck(indx,:)=[];
                end
            end
        elseif ~isempty(Xfinal{i})
            Xfinal_Store{i} = Xfinal{i};
            pos_a_loop(i,:) = Xfinal{i}(1:2,1)';
            v_a_loop(i,:)   = Xfinal{i}(3:4,1)';
            Xfinal{i}(:,1)=[];
        end
    end
        
     
     
    n_rounds_loop = n_rounds_loop - 1;
    simu_time_loop = simu_time_loop - time_step;
    time_start_loop = time_start_loop + time_step;
    tf_t_loop = tf_t_loop - time_step;
    
    
    if ~isempty(completed_agents)
        %%%%%%%%%%%%%%%%%%% for Update Agents %%%%%%%%%%%%%%%%%%%%%
        nonAllocatedAgents = find(cellfun(@isempty,p_GCAA));
        if ~isempty(nonAllocatedAgents)
            na_new = length(completed_agents)+length(nonAllocatedAgents);
            Agents.Pos = pos_a_loop([completed_agents nonAllocatedAgents],:);
        else
            na_new = length(completed_agents);
            Agents.Pos = pos_a_loop(completed_agents,:);
        end

        v_a_loop = zeros(na_new,2);
        Agents.N = na_new;
        Agents.Lt = Lt * ones(1,na_new);
        Agents.previous_task = zeros(na_new,1);
        Agents.previous_winnerBids = zeros(na_new,1);
        Agents.rin_task = zeros(na_new,2);
        Agents.vin_task = zeros(na_new,2);
        Agents.kdrag = kdrag;
        Agents.v_a=v_a_loop;
        
        %%%%%%%%%%%%%%%%%%%%%% for Update Targets for GCAA %%%%%%%%%%%%%%%%%%
        tmpi = setdiff(cell2mat(p_GCAA),completed_tasks_round);
        [~,indtmp1] = ismember(tmpi,Tasks.Pos(:,3));
        indtmp1(indtmp1==0)=[];
        Tasks.r_bar(indtmp1,:) = [];
        %         Tasks.prob_a_t = Tasks.prob_a_t(completed_agents,:);
        %         Tasks.prob_a_t(:,indtmp1) = [];
        Tasks.Pos(indtmp1,:) = [];
        Tasks.Speed(indtmp1,:) = [];
        Tasks.N = size(Tasks.Pos,1);
        Tasks.tf(indtmp1,:) = [];
        Tasks.lambda = lambda;
        Tasks.task_type(indtmp1,:) = [];
        Tasks.task_type1(indtmp1,:) = [];
        Tasks.tloiter(indtmp1,:) = [];
        Tasks.radius(indtmp1,:) = [];
        Tasks.radius_t_2(indtmp1,:) = [];
        Tasks.angle(indtmp1) = [];
        Tasks.restCheck(indtmp1) = [];
        
        if isempty(Tasks.Pos) && ~isempty(Tasks1.Pos)
           Tasks = Tasks1; 
        end       
        
        %         Tasks.prob_a_t = rand(na_new,nt);        
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