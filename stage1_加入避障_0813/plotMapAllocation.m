function plotMapAllocation(X, na, color, name)

    for i = 1:na      
%         xx = reshape(X(:,i,:),[4,n_rounds+1]);
        xx=X{i};
%         xx = reshape(X(:,i,:),[4,size(X,3)]);
        if ~isempty(xx)
%             plot(xx(1,:),xx(2,:), ':', 'Color', color(i,:), 'LineWidth', 2, 'DisplayName', name);
            plot(xx(1,:),xx(2,:),'ko','LineWidth',1,'MarkerSize',1, 'Color', color(i,:));
        end
    end

end