function H = fitRectangle(im, regions_centroid, debug)
    gradMask = zeros(size(im));
    gradMask(sub2ind(size(gradMask), floor(regions_centroid(:,2)), floor(regions_centroid(:,1)))) = 255;
    
    foregroundMask = bwconvhull(gradMask);
    edgeMask = edge(foregroundMask, 'Canny');
    edgeMaskDilated = imdilate(edgeMask,strel('square', 2));
    
    % Use Hough transform to find the borders of the card
    [H, T, R] = hough(edgeMaskDilated, 'Theta', -90:1:89);
    P = houghpeaks(H, 100, 'Threshold', 0.3 * max(H(:)));
    ini_lines = houghlines(edgeMaskDilated, T, R, P, 'FillGap', min(size(im))*0.1, 'MinLength', min(size(im))*0.025);
    
    % Extract lines information from the lines structure and compute the
    % cartesian length 
    rho = []; theta = [];
    p1 = []; p2 = []; len = [];
    for i=1:length(ini_lines)
        theta = [theta ini_lines(i).theta];
        rho = [rho ini_lines(i).rho];
        p1 = [p1; ini_lines(i).point1];
        p2 = [p2; ini_lines(i).point2];
        len = [len norm(p1(end,:)-p2(end,:))];
    end
    
    % Remove duplicated lines
    [~,unique_ids,~]  = unique([p1 p2],'rows','stable');    
    lines = ini_lines(unique_ids);   
    rho = rho(unique_ids);
    theta = theta(unique_ids);
    p1 = p1(unique_ids,:);
    p2 = p2(unique_ids,:);
    len = len(unique_ids);
    
    % Compute distance between lines in order to prune out the too close
    % lines
    lines_dist = zeros(length(lines));
    for i=1:length(lines)
        for ii=i+1:length(lines)
            p1_l1_l2 = point_to_line(p1(ii,:),p1(i,:),p2(i,:));
            p2_l1_l2 = point_to_line(p2(ii,:),p1(i,:),p2(i,:));
            p1_l2_l1 = point_to_line(p1(i,:),p1(ii,:),p2(ii,:));
            p2_l2_l1 = point_to_line(p2(i,:),p1(ii,:),p2(ii,:));
            
            lines_dist(i,ii) = max([p1_l1_l2, p2_l1_l2, p1_l2_l1, p2_l2_l1]);
        end
    end
    
    % The maximum line_dist must be carefully set in order to merge
    % the lines, a fails can be due by this too small parameter
    groups = {};
    logical_thresholded_dist = (lines_dist <= min(size(im))*0.1 & lines_dist > 0);
    for i=1:length(lines)
        if isempty([groups{:}]) || ~ismember(i,[groups{:}])
            groups{end+1} = [i find(logical_thresholded_dist(i,:))];
        end
    end
    
    ids = [];
    for i=1:length(groups)
        [~, id] = max(len(groups{i}));
        ids = [ids groups{i}(id)];
    end
    
    theta = theta(ids);
    rho = rho(ids);
    
    % Sort by angles and convert to radians    
    [~,order] = sort(abs(theta));
    coefficients = [cos(theta' / 180 * pi), sin(theta' / 180 * pi), -rho'];

    % If i have more than four lines the first two are the parallel and the
    % last two are the perpendicular to it    
    if(length(ids) < 4)
        H = [];
        return;
    end
    
    % start with smallest angle
    L1 = coefficients(order(1),:);
    % parallel to L1
    L2 = coefficients(order(2),:);
    % perpendicular to L1 and L2
    L3 = coefficients(order(end-1),:);
    % parallel to L3
    L4 = coefficients(order(end),:);
    
    p = zeros(4,3);
    
    % p1
    p(1,:) = cross(L1,L3);
	% p2
    p(2,:) = cross(L2,L3);
    % p3
    p(3,:) = cross(L2,L4);
    % p4
    p(4,:) = cross(L1,L4);

    p = p(:,1:2) ./ [p(:,3), p(:,3)];
    
    v = p(2,:) - p(1,:); 
    w = p(3,:) - p(1,:);
    a = (v(1)*w(2) - w(1)*v(2)) / 2; % signed area of triangle (p1,p2,p3)
    if (a < 0)
        p = p(end:-1:1,:); % reverse vertex order
    end
    
    edgeLen = [
        norm(p(1,:) - p(2,:));
        norm(p(2,:) - p(3,:));
        norm(p(3,:) - p(4,:));
        norm(p(4,:) - p(1,:))
    ];
    
    % flag which ones are at a short edge
    sortedEdgeLen = sort(edgeLen);
    startsShortEdge = (edgeLen' <= sortedEdgeLen(2));

    % find the one that is closest to top left corner of image
    idx = find(startsShortEdge,2);
    
    if isempty(idx)
        H = [];
        return;
    end
    
    if (norm(p(idx(1),:)) < norm(p(idx(2),:)))
        p1Idx = idx(1);
    else
        p1Idx = idx(2);
    end
    
    % reorder to start at right vertex
    order = mod(p1Idx - 1 : p1Idx + 2, 4) + 1;
    corners = p(order,:);

    % Re-order corners this way: tl, tr, br, bl
    % Assume that the tl corner is closest to 1,1, etc.
    imageCorners = [
        1,          1;
        size(im, 2),1;
        size(im, 2),size(im, 1);
        1,          size(im, 1)];
    cornersTmp = [];

    for i = 1 : 4
        cornersVector = corners - repmat(imageCorners(i, :), size(corners, 1), 1);
        dist = (cornersVector(:, 1).^2 + cornersVector(:, 2).^2) .^ 0.5;
        [~, ind] = min(dist);
        cornersTmp(i, :) = corners(ind, :);
    end
    corners = cornersTmp;
    
    % Measure the skewed widths & heights
    heightL = norm(corners(1,:) - corners(4,:));
    heightR = norm(corners(2,:) - corners(3,:));
    widthT = norm(corners(1,:) - corners(2,:));
    widthB = norm(corners(3,:) - corners(4,:));

    % Set up the target image dimensions
    % Use the maximum of skewed width and height
    % to approxmate the target dimensions
    imNewHeight = max([heightL, heightR]);
    imNewWidth  = max([widthT, widthB]);
    cornersNew = [
        (size(im,1)/2)-(imNewWidth/2),              (size(im,1)/2)-(imNewHeight/2);
        (size(im,1)/2)-(imNewWidth/2) + imNewWidth, (size(im,1)/2)-(imNewHeight/2);
        (size(im,1)/2)-(imNewWidth/2) + imNewWidth, (size(im,1)/2)-(imNewHeight/2) + imNewHeight;
        (size(im,1)/2)-(imNewWidth/2),              (size(im,1)/2)-(imNewHeight/2) + imNewHeight];
    
    % Visualization
    if debug
        figure(777);
        clf;
        imagesc(edgeMaskDilated);
        colormap gray;
        hold on;
        for l = 1:length(lines)
            plot([lines(l).point1(1) lines(l).point2(1)], ...
                 [lines(l).point1(2) lines(l).point2(2)],'r');
        end
        
        for l = 1:length(ids)
            plot([lines(ids(l)).point1(1) lines(ids(l)).point2(1)], ...
                 [lines(ids(l)).point1(2) lines(ids(l)).point2(2)],'b');
        end
        
        hold on;
        plot(corners(:,1),corners(:,2),'bo');
        plot(cornersNew(:,1),cornersNew(:,2),'go');
        title('Fitted rectangle with rectified corners')
        pause;
    end
    
    % Compute the homography matrix
    try
        H = estimateGeometricTransform(corners, cornersNew,'projective');
        if debug
            figure(7744);
            imshow(imwarp(im, H, 'OutputView', imref2d(size(im))));
        end
    catch
       H = []; 
    end
end