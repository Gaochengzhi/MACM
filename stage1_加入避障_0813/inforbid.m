function in = inforbid(p_forbidArea,point)

size_poly = polyarea(p_forbidArea(1,:),p_forbidArea(2,:));
size_rectan = 0;
for i = 1 : length(p_forbidArea)-1
    rectan = [point;
        p_forbidArea(:,i)';
        p_forbidArea(:,i+1)';]';
    size_rectan = size_rectan + polyarea(rectan(1,:),rectan(2,:));
end
rectan = [point;
    p_forbidArea(:,length(p_forbidArea))';
    p_forbidArea(:,1)';]';
size_rectan = size_rectan + polyarea(rectan(1,:),rectan(2,:));

in = (size_rectan - size_poly) <= 0.08*size_poly;

end