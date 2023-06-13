function plotMapAllocation_stage2(X, na, color, name)

    for i = 1:na      
%         xx = reshape(X(:,i,:),[4,n_rounds+1]);
        xx=X{i};
%         xx = reshape(X(:,i,:),[4,size(X,3)]);
        if ~isempty(xx)
            plot(xx(1,:),xx(2,:), ':', 'Color', color(i,:), 'LineWidth', 2, 'DisplayName', name);
        end
    end

end