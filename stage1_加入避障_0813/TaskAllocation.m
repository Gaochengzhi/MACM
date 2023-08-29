function [p_alloc, taskInd]=TaskAllocation(Agents, Tasks,agentNo)
na = Agents.N;
nt = Tasks.N;
restCheck = Tasks.restCheck;
UtilMatrix = zeros(na,nt);
dist = zeros(na,nt);
tmp = zeros(na,1);
idx = Tasks.Pos(:,3);


for i=1:na
    for j=1:nt
        dist(i,j) = norm(Agents.Pos(i,1:2) - Tasks.Pos(j,1:2));
    end
    tmp(i) = sum(dist(i,:));
end

%%%%%%%%%%% normalization %%%%%%%%%%%%%%%%%%%
dist = dist./repmat(tmp,1,nt);

%%%%%%%%%%%% Rest Check %%%%%%%%%%%%%%%%%%
% UtilMatrix = 1./dist.*repmat(restCheck',na,1)+Tasks.prob_a_t;
UtilMatrix = 1./dist+Tasks.prob_a_t(agentNo,idx);
[p_alloc,~] = munkres(-UtilMatrix);
% taskInd = Tasks.Pos(p_alloc,3);

for i=1:length(p_alloc)
    if p_alloc(i)==0
        taskInd(i)=0;
    else
        taskInd(i) = Tasks.Pos(p_alloc(i),3);
    end
end
