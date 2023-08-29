%%Optimal control solution
% for a double integrator dynamics and minimizing the square of the input command along the path
% Params: p_GCAA: task allocation
% Output: X: (n_rounds x na x 4) matrix corresponding to the (x,y) position and velocity of
%            each agents for each round (n_rounds = tf/time_step)
function [Xfinal,xmax] = fly2waypoints(na, pos_a,waypoints,n_rounds)
X = zeros(4, na, n_rounds+1);
xmax=0;
for i=1:na
    wayP = waypoints{i};
    LineNo = size(wayP,1);
    dis_left = norm(pos_a(i,:)- wayP(1,1:2));
    dis_right = norm(pos_a(i,:)- wayP(1,3:4));
    startfromLeft = dis_left-dis_right<0;
    
    tmp0 = wayP;
    tmp1 = wayP(1:2:end,:);  %%% odd
    tmp2 = wayP(2:2:end,:);  %%% even
    tmp1(:,[1,3]) = tmp1(:,[3,1]);
    tmp1(:,[2,4]) = tmp1(:,[4,2]);
    tmp2(:,[1,3]) = tmp2(:,[3,1]);
    tmp2(:,[2,4]) = tmp2(:,[4,2]);
    
    if startfromLeft
        tmp0(2:2:end,:) = tmp2;
    else
        tmp0(1:2:end,:) = tmp1;
    end
    
    organizedWaypoints{i} = [pos_a(i,:);reshape(reshape(tmp0',[],1),2,[])'];
    oWp = organizedWaypoints{i};
    %%%%%%%%%%%%%%%% Fly to waypoints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    X(1:2,i,1)=oWp(1,:);
    
    K = 0;
    
    
    for j = 1:length(oWp)-1
        dis = oWp(j+1,:)-oWp(j,:);
        d = norm(dis);
        v(:,j) = [(dis/d*20)';0;0];
        K(j)=max(floor(dis(1)/v(1,j)),floor(dis(2)/v(2,j)));
    end
    
    
    for m=1:length(K)
        if m==1
            iter0=1;
        else
            iter0 = iter0+K(m-1);
        end
        iterEnd = iter0+K(m)-1;
        for k=iter0:iterEnd
            X(:, i, k+1) = X(:, i, k) + v(:,m);
        end
    end
    
    tmp = X(:,i,:);
    tmp( :, all(~tmp,1) ) = [];
    Xfinal{i} = tmp;
    
    xmax = max(xmax,size(Xfinal{i},2));
end
end


