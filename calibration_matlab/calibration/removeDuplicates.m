function finalPoints = removeDuplicates(inputPoints, distance)

    dists = triu(pdist2(inputPoints, inputPoints) <= distance);
    finalPoints = [];
    
    while size(dists,1) > 0 && any(dists(:))

        similar = find(dists(1,:) == 1);
        if ~isempty(similar)
            finalPoints = [finalPoints; mean(inputPoints(similar,:), 1)];

            dists(similar,:) = [];
            dists(:,similar) = [];
            inputPoints(similar,:) = [];
        end
    end
    
end