function [Homography,BoundingBox,Points,SquareSize] = patternExtraction(I, ...
    columns,rows,threshold,solidity,skip_homography,detect_edges,debug)

    % Edge detection
    if detect_edges
        Iblur = medfilt2(I);
        BW = edge(Iblur,'Sobel');
        SE = strel('square', 2);
        BW = imdilate(BW,SE);
        FilteredImg = BW;
    else
        FilteredImg = I;
    end
    
    % Image rectification
    [Homography, centroids, meanDiameter] = findRig(FilteredImg, columns, rows, threshold, solidity, skip_homography, debug);
    
    % If we can't detect board skip image
    if isempty(Homography) && ~skip_homography
        BoundingBox = [];
        Points = [];
        SquareSize = -1;
        return;
    end

    % extract mean calibration pattern centroid
    if skip_homography
        UnPerspectI = I;
        rectifiedCentroids = centroids;
    else
        try
            UnPerspectI = imwarp(I, Homography, 'OutputView', imref2d(size(I)));
        catch ME
            disp(ME);
            BoundingBox = [];
            Points = [];
            SquareSize = -1;
            return;
        end
        UnPerspectI(isnan(UnPerspectI)) = 0;

        % project centroids on the rectified plane
        tmp = [centroids ones(size(centroids,1),1)] * Homography.T;
        tmp = tmp ./ tmp(:,3);
        rectifiedCentroids = tmp(:,1:2);        
    end
    
    if(isempty(rectifiedCentroids))
        BoundingBox = [];
        Points = [];
        SquareSize = -1;
        return;        
    end

    % remove out of pattern and duplicated points
    % build geometry for homography estimation
    tmpWholeImagePoints = removeDuplicates(rectifiedCentroids, meanDiameter);   

    % pattern extents estimation
    min_x = min(tmpWholeImagePoints(:,1) - meanDiameter/2);
    max_x = max(tmpWholeImagePoints(:,1) + meanDiameter/2);
    min_y = min(tmpWholeImagePoints(:,2) - meanDiameter/2);
    max_y = max(tmpWholeImagePoints(:,2) + meanDiameter/2);       

    if debug
        figure(4);
        clf;
        imshow(UnPerspectI);
        hold on;
        plot(tmpWholeImagePoints(:,1),tmpWholeImagePoints(:,2),'bx');
        rectangle('Position',[min_x min_y (max_x-min_x) (max_y-min_y)]);
        title('Pattern extraction unperspected partial detection');
        pause;
    end

    % estimating centroids distance over width
    px_coord = sortrows(floor(tmpWholeImagePoints), 2);        
    counter = 1;        
    for k=1:size(px_coord,1) - 1
        if abs(px_coord(k,2)-px_coord(k+1,2)) >= meanDiameter                
            px_coord(counter:k,2) = mean(px_coord(counter:k,2));
            counter = k + 1;
        end
    end
    px_coord(counter:k+1,2) = mean(px_coord(counter:k+1,2));    
    px_coord = sortrows(px_coord, [2 1]);
    x_candidates = find(sum(px_coord(:,2) == px_coord(:,2)') > 1);
    
    if(isempty(x_candidates))
        BoundingBox = [];
        Points = [];
        SquareSize = -1;
        return;
    end
    
    meanDiameterRefinedX = abs((px_coord(x_candidates(2),1) - px_coord(x_candidates(1),1))/2);

    % estimating centroids distance over height
    py_coord = sortrows(floor(tmpWholeImagePoints), 1);
    counter = 1;
    for k=1:size(py_coord,1) - 1
        if abs(py_coord(k,1)-py_coord(k+1,1)) >= meanDiameter                
            py_coord(counter:k,1) = mean(py_coord(counter:k,1));
            counter = k + 1;
        end
    end
    py_coord(counter:k+1,1) = mean(py_coord(counter:k+1,1));
    py_coord = sortrows(py_coord, [1 2]);
    y_candidates = find(sum(py_coord(:,1) == py_coord(:,1)') > 1);
    
    if(isempty(y_candidates))
        BoundingBox = [];
        Points = [];
        SquareSize = -1;
        return;
    end

    meanDiameterRefinedY = abs((py_coord(y_candidates(1),2) - py_coord(y_candidates(2),2))/2);

    % Assign and return
    BoundingBox = double([min_x min_y (max_x - min_x) (max_y - min_y)]);
    Points = tmpWholeImagePoints;
    SquareSize = [double(meanDiameterRefinedX) double(meanDiameterRefinedY)];
end