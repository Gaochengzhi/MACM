%% Optimal control solution
% for a double integrator dynamics and minimizing the square of the input command along the path
% Params: p_GCAA: task allocation
% Output: X: (n_rounds x na x 4) matrix corresponding to the (x,y) position and velocity of
%            each agents for each round (n_rounds = tf/time_step)
function [Xfinal, completed_tasks, J, J_to_completion_target] = OptimalControlSolution_stage2(Agents, Tasks, p_GCAA, time_step, n_rounds, kdrag)
pos_a = Agents.Pos;
v_a = Agents.v_a;
pos_t = Tasks.Pos(:,1:2);
v_t = Tasks.Speed;
radius_t = Tasks.radius;
tf_t=Tasks.tf;
tloiter_t=Tasks.tloiter;
na = Agents.N;
targets_angle = Tasks.angle;
targets_restCheck = Tasks.restCheck;
flag_changeTargets = Tasks.flag_changeTargets;

pos_t_curr=[];
pos_a_curr=[];
Xfinal = cell(na,1);
X = zeros(4, na, n_rounds+1);
J = [zeros(1,na)];
J_to_completion_target = [zeros(1,na)];
A = [zeros(2,2) eye(2,2); zeros(2,2) zeros(2,2)];
B = [zeros(2,2); eye(2,2)];
completed_tasks = [];
for i = 1:na %%逐个计算每个Agent的状态量
    v_a(i,:) = zeros(size(v_a(i,:)));
    X(:, i, 1) = [pos_a(i,:) v_a(i,:)];
    if (isempty(p_GCAA{i}) || p_GCAA{i} == 0)
        p_GCAA{i} = [];
        for k = 1:size(X,3)-1
            if k == 1
                u = - kdrag * X(3:4, i, k);
                X(:, i, k+1) = X(:, i, k) + time_step * (A * X(:, i, k) + B * u);
            else
                X(:, i, k+1) = X(:, i, k);
            end
        end
        continue;
    end

    for j = 1:size(p_GCAA{i},2)
        k = 0;
        ind_task = p_GCAA{i}(j);
        tf = tf_t(ind_task);
        if j > 1
            tf = tf - tf_t(p_GCAA{i}(j-1));
        end
        pos_t_curr = pos_t(ind_task,:)';
        v_t_curr = v_t(ind_task,:)';
        pos_a_curr = X(1:2, i, k+1);
        v_a_curr = X(3:4, i, k+1);
        
        J_to_completion_target_curr = 0;
        %         if tf > tloiter_t(ind_task) + time_step
        %             t_to_target = 0:time_step:(tf-tloiter_t(ind_task));
        %             %                 [uparams, rparams, vparams, tparams, J_to_completion_target_curr] = ComputeCommandParamsWithVelocity(pos_a_curr, v_a_curr, Agents.rin_task(i,:)', Agents.vin_task(i,:)', tf - tloiter_t(ind_task), t_to_target, Agents.kdrag);%%只有uparams是后面用到的
        %         end
        J_to_completion_target(i) = J_to_completion_target_curr;
        R = radius_t(ind_task);
        norm_vt = 2*pi * R / tloiter_t(ind_task);
        norm_a = norm_vt^2 / R;

        if targets_angle(ind_task) >= 270 && targets_angle(ind_task) <= 360
            exp_angle = 450 - targets_angle(ind_task);
        else
            exp_angle = 90 - targets_angle(ind_task);
        end    %%%%20230610修改
        
        if tloiter_t(ind_task) > 0 && tf > 0
            J_to_completion_target(i) = J_to_completion_target(i) + 1/2 * norm_a^2 * min(tloiter_t(ind_task), tf);
        end
        
        t = 0;
        %             while t + time_step <= tf
        
        pos_t_curr_app(1,1) = pos_t_curr(1,1) + R * cosd(exp_angle);
        pos_t_curr_app(2,1) = pos_t_curr(2,1) + R * sind(exp_angle);
        
        %         while norm(pos_t_curr - X(1:2, i, k+1)) > 53  %% 设置终点触发条件
        
        while norm(pos_t_curr - X(1:2, i, k+1)) > 53 %% 设置终点触发条件
            u = 0;
            angle = X(1:2, i, k+1) - pos_t_curr;  %%%%20230610修改
            if norm(pos_t_curr - X(1:2, i, k+1)) < R+3 && abs(abs(atan2(angle(2,1),angle(1,1))*180/pi) - abs(exp_angle)) < 5  %%朝真正的终点飞行  %%%20230610修改
                r_target_circle = pos_t_curr - X(1:2, i, k+1);
                d = norm(r_target_circle);
                curr_velocity = zeros(size(X(:, i, k+1)));
                curr_velocity(1:2) = r_target_circle / d;
                X(:, i, k+2) = X(:, i, k+1) + time_step * curr_velocity * 9 * 0.5144444;
            elseif norm(pos_t_curr_app - X(1:2, i, k+1)) > 3 % && norm(pos_t_curr-X(1:2, i, k+1))>R   %%朝外接点飞行
                r_target_circle = pos_t_curr_app - X(1:2, i, k+1);
                d = norm(r_target_circle);
                curr_velocity = zeros(size(X(:, i, k+1)));
                curr_velocity(1:2) = r_target_circle / d;
                X(:, i, k+2) = X(:, i, k+1) + time_step * curr_velocity * 20 * 0.5144444;
                tmp=k+1;
            end
            
            t = t + time_step;
            k = k + 1;
            
            if k == 0
                J(i) = 1/2 * norm(u)^2 * time_step;
            end
        end
        
    end
    
    if isempty(pos_t_curr) || isempty(pos_a_curr)
        pos_t_curr=0;
        pos_a_curr=0;
    end
    
    if  norm(pos_t_curr - pos_a_curr) <= 53
        completed_tasks = [completed_tasks p_GCAA{i}];
    end
    
    
    if flag_changeTargets(ind_task)<=1 && targets_restCheck(ind_task)<=1
        tmp = X(:,i,:);
        tmp( :, all(~tmp,1) ) = [];
        Xfinal{i} = tmp;
    elseif flag_changeTargets(ind_task)>1 && targets_restCheck(ind_task)<=1    
        len = k+1-tmp+1;
        Xtmp = X(:,i,tmp:k+1);
        Xtmp1 = flip(Xtmp(:,:,1:end-1),3);
        X(:,i,k+2:k+2+len-2)= Xtmp1;
        
        tmp = X(:,i,:);
        tmp( :, all(~tmp,1) ) = [];
        Xfinal{i} = tmp;
        
    elseif flag_changeTargets(ind_task)<=1 && targets_restCheck(ind_task)>1
        len = k+1-tmp+1;
        Xtmp = X(:,i,tmp:k+1);
        Xtmp1 = flip(Xtmp(:,:,1:end-1),3);
        Xtmp2 = cat(3,Xtmp1,Xtmp(:,:,2:end));
        Xtmp3 = repmat(Xtmp2,1,1,targets_restCheck(ind_task)-1);
        X(:,i,k+2:k+1+(targets_restCheck(ind_task)-1)*(2*len-2))=Xtmp3;
        
        tmp = X(:,i,:);
        tmp( :, all(~tmp,1) ) = [];
        Xfinal{i} = tmp;
    elseif flag_changeTargets(ind_task)>1 && targets_restCheck(ind_task)>1
        len = k+1-tmp+1;
        Xtmp = X(:,i,tmp:k+1);
        Xtmp1 = flip(Xtmp(:,:,1:end-1),3);
        Xtmp2 = cat(3,Xtmp1,Xtmp(:,:,2:end));
        Xtmp3 = repmat(Xtmp2,1,1,targets_restCheck(ind_task)-1);
        X(:,i,k+2:k+1+(targets_restCheck(ind_task)-1)*(2*len-2)+len-1)=cat(3,Xtmp3,Xtmp1);
        
        tmp = X(:,i,:);
        tmp( :, all(~tmp,1) ) = [];
        Xfinal{i} = tmp;        
    end
    
end
end