function size_rectan = size_rectan(position , Area)

size_rectan = 0;
for i = 1 : length(Area)-1
    rectan = [position;
        Area(:,i)';
        Area(:,i+1)';]';
    size_rectan = size_rectan + polyarea(rectan(1,:),rectan(2,:));
end
rectan = [position;
    Area(:,length(Area))';
    Area(:,1)';]';
size_rectan = size_rectan + polyarea(rectan(1,:),rectan(2,:));

end