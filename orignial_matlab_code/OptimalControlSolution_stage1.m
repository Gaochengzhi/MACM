%% Optimal control solution
% for a double integrator dynamics and minimizing the square of the input command along the path
% Params: p_GCAA: task allocation
% Output: X: (n_rounds x na x 4) matrix corresponding to the (x,y) position and velocity of
%            each agents for each round (n_rounds = tf/time_step)
function [X, completed_tasks] = OptimalControlSolution_stage1(pos_a, v_a, pos_t, v_t, radius_t, p_GCAA, Agents, tf_t, tloiter_t, time_step, n_rounds, na, kdrag)
pos_t_curr=[];
pos_a_curr=[];
X = zeros(4, na, n_rounds+1);
% J = [zeros(1,na)];
% J_to_completion_target = [zeros(1,na)];
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

%         J_to_completion_target_curr = 0;
        %         if tf > tloiter_t(ind_task) + time_step
        %             t_to_target = 0:time_step:(tf-tloiter_t(ind_task));
        %             %                 [uparams, rparams, vparams, tparams, J_to_completion_target_curr] = ComputeCommandParamsWithVelocity(pos_a_curr, v_a_curr, Agents.rin_task(i,:)', Agents.vin_task(i,:)', tf - tloiter_t(ind_task), t_to_target, Agents.kdrag);%%只有uparams是后面用到的
        %         end
%         J_to_completion_target(i) = J_to_completion_target_curr;
        R = radius_t(ind_task);
        norm_vt = 2*pi * R / tloiter_t(ind_task);
        norm_a = norm_vt^2 / R;

%         if tloiter_t(ind_task) > 0 && tf > 0
%             J_to_completion_target(i) = J_to_completion_target(i) + 1/2 * norm_a^2 * min(tloiter_t(ind_task), tf);
%         end

        t = 0;
        %             while t + time_step <= tf

        pos_t_curr_app(1,1) = pos_t_curr(1,1) + R * cosd(15);
        pos_t_curr_app(2,1) = pos_t_curr(2,1) + R * sind(15);
        %         circle_intersection = pos_t_curr + R * (X(1:2, i, k+1)-pos_t_curr) / norm(X(1:2, i, k+1)-pos_t_curr);

        %         while norm(pos_t_curr_app - X(1:2, i, k+1)) > 3 && dot((circle_intersection - X(1:2, i, k+1)),(pos_t_curr - X(1:2, i, k+1))) > 0 %% 朝目标外接点飞行
        while norm(pos_t_curr - X(1:2, i, k+1)) > 40  %% 设置终点触发条件
            u = 0;
            angle = pos_t_curr - X(1:2, i, k+1);
            %             if norm(pos_t_curr - X(1:2, i, k+1)) < R+3 && abs(atan2(angle(2,1),angle(1,1))*180/pi - (-165)) < 5 %%朝真正的终点飞行
            %                r_target_circle = pos_t_curr - X(1:2, i, k+1);
            %                d = norm(r_target_circle);
            %                curr_velocity = zeros(size(X(:, i, k+1)));
            %                curr_velocity(1:2) = r_target_circle / d;
            %                X(:, i, k+2) = X(:, i, k+1) + time_step * curr_velocity * 90;
            %             elseif norm(pos_t_curr_app - X(1:2, i, k+1)) > 3  %%朝外接点飞行
            %                r_target_circle = pos_t_curr_app - X(1:2, i, k+1);
            %                d = norm(r_target_circle);
            %                curr_velocity = zeros(size(X(:, i, k+1)));
            %                curr_velocity(1:2) = r_target_circle / d;
            %                X(:, i, k+2) = X(:, i, k+1) + time_step * curr_velocity * 180;
            %             end

            r_target_circle = pos_t_curr - X(1:2, i, k+1);
            d = norm(r_target_circle);
            curr_velocity = zeros(size(X(:, i, k+1)));
            curr_velocity(1:2) = r_target_circle / d;
            X(:, i, k+2) = X(:, i, k+1) + time_step * curr_velocity * 750;

            t = t + time_step;
            k = k + 1;

%             if k == 0
%                 J(i) = 1/2 * norm(u)^2 * time_step;
%             end

        end

    end
    % If it reaches the target
    %         if k == 1 && sum(completed_tasks == p_GCAA{i}) == 0 %%k==0?
    %         if k == 0 && sum(completed_tasks == p_GCAA{i}) == 0 %%k==0?

    if isempty(pos_t_curr) || isempty(pos_a_curr)
        pos_t_curr=0;
        pos_a_curr=0;
    end

    if  norm(pos_t_curr - pos_a_curr) <= 40
        completed_tasks = [completed_tasks p_GCAA{i}];
    end


    for k2 = k+2:n_rounds+1
        X(:,i,k2) = X(:,i,k+1);
    end

end
end