function [wholePoints, BoundingBox] = fillPattern(I, partialPoints, squareSizeEstimation, relax, columns, rows, debug)

    % defult value
    wholePoints = [];
    BoundingBox = [-1 -1 -1 -1];
    detectedPoints = [];
        
    for i=1:length(partialPoints)
        % check for squares on a cross over all points [top bottom right
        % left]
        vertices = zeros(4,2);
        vertices(1,:) = partialPoints(i,:) + [0 2*squareSizeEstimation(2)];
        vertices(2,:) = partialPoints(i,:) - [0 2*squareSizeEstimation(2)];
        vertices(3,:) = partialPoints(i,:) + [2*squareSizeEstimation(1) 0];
        vertices(4,:) = partialPoints(i,:) - [2*squareSizeEstimation(1) 0];
        
        % building subregion for each partial point
        for k=1:size(vertices,1)
            
            % Instead of filling the missing pattern, the function is called anyway
            % in order to refine the estimation of the centroid.
            
            % if the vertex is too close to an existing point, skip
            % if min(min(pdist2(single(vertices(k,:)),single(partialPoints)))) <= min(squareSizeEstimation)
            %    continue;
            % end
            
            % retrive mean and std of the paper grey level
            paper_mask_mean = []; paper_mask_std = [];

            paper_mask_r = vertices(k,:) + [squareSizeEstimation(1) 0];
            paper_mask_l = vertices(k,:) + [-squareSizeEstimation(1) 0];
            paper_mask_u = vertices(k,:) + [0 squareSizeEstimation(2)];
            paper_mask_d = vertices(k,:) + [0 -squareSizeEstimation(2)];

            bbox_r = [(paper_mask_r(1) - squareSizeEstimation(1)/4) (paper_mask_r(2) - squareSizeEstimation(2)/4) squareSizeEstimation(1)/2 squareSizeEstimation(2)/2];
            Icrop = imcrop(I, bbox_r);
            paper_mask_mean = [paper_mask_mean mean(mean(Icrop(:)))];
            paper_mask_std = [paper_mask_std std(double(Icrop(:)))];

            bbox_l = [(paper_mask_l(1) - squareSizeEstimation(1)/4) (paper_mask_l(2) - squareSizeEstimation(2)/4) squareSizeEstimation(1)/2 squareSizeEstimation(2)/2];
            Icrop = imcrop(I, bbox_l);
            paper_mask_mean = [paper_mask_mean mean(mean(Icrop(:)))];
            paper_mask_std = [paper_mask_std std(double(Icrop(:)))];

            bbox_u = [(paper_mask_u(1) - squareSizeEstimation(1)/4) (paper_mask_u(2) - squareSizeEstimation(2)/4) squareSizeEstimation(1)/2 squareSizeEstimation(2)/2];
            Icrop = imcrop(I, bbox_u);
            paper_mask_mean = [paper_mask_mean mean(mean(Icrop(:)))];
            paper_mask_std = [paper_mask_std std(double(Icrop(:)))];

            bbox_d = [(paper_mask_d(1) - squareSizeEstimation(1)/4) (paper_mask_d(2) - squareSizeEstimation(2)/4) squareSizeEstimation(1)/2 squareSizeEstimation(2)/2];
            Icrop = imcrop(I, bbox_d);
            paper_mask_mean = [paper_mask_mean mean(mean(Icrop(:)))];
            paper_mask_std = [paper_mask_std std(double(Icrop(:)))];

            if debug
                figure(86)
                clf;
                imshow(I)
                hold on
                plot(vertices(k,1), vertices(k,2), 'r*')
                title('Pattern filling unperspected interation detection');
                
                rectangle('Position', bbox_r, 'EdgeColor','g', 'LineWidth', 1);
                rectangle('Position', bbox_l, 'EdgeColor','g', 'LineWidth', 1);
                rectangle('Position', bbox_u, 'EdgeColor','g', 'LineWidth', 1);
                rectangle('Position', bbox_d, 'EdgeColor','g', 'LineWidth', 1);
            end            
            
            % compute global mean and global std
            paper_mask_mean = mean(paper_mask_mean);
            paper_mask_std = mean(paper_mask_std);
            
            bbox = round([(vertices(k,1) - squareSizeEstimation(1)) (vertices(k,2) - squareSizeEstimation(2)) 2*squareSizeEstimation(1) 2*squareSizeEstimation(2)]);
            Icrop = imcrop(I, bbox);
            
            if(sum(sum(isnan(Icrop))) > 0 || size(Icrop,1) == 0)
                continue;
            end
            
            try
                [L,C] = adaptcluster_kmeans(Icrop(:));
                clusteredIcrop = reshape(L, size(Icrop));
                [C_sorted, color_id] = sort(C);
            catch
                continue;
            end
            
            % compute the difference between the paper and the each cluster
            % if the difference is below the std mark the same clusters
            grey_val = abs(C_sorted - paper_mask_mean);
            same_within_std = grey_val < paper_mask_std;
            grey_val(same_within_std) = 0;
            [~, paper_id] = min(grey_val);          
            
            % mark the clusters if they are close each other less than the
            % std of paper mask
            same_within_center_std = 1:length(grey_val);
            for n =1:length(grey_val)
                for m=n:length(grey_val)
                    if abs(grey_val(n) - grey_val(m)) < paper_mask_std
                        same_within_center_std(n) = m;
                    end
                end
            end
            
            % assign new classes based on the paper mask cluster id
            new_indexing = {};
            for n=1:length(color_id)
                new_indexing{n} = find(clusteredIcrop == n);
            end
            
            for n=1:length(new_indexing)
                clusteredIcrop(new_indexing{n}) = find(color_id == n);
            end
            
            % effectivly collapse the clusters which are close to the
            % paper mask under the std
            for n=1:length(same_within_std)
                if same_within_std(n) == true
                    clusteredIcrop(clusteredIcrop == n) = paper_id;
                end
            end

            % effectivly collapse the clusters which are close each other
            % less than the paper mask std
            for n=1:length(same_within_center_std)
                if same_within_center_std(n) ~= n
                    clusteredIcrop(clusteredIcrop == n) = same_within_center_std(n);
                end
            end

            lowBinarize = zeros(size(Icrop));
            lowBinarize(clusteredIcrop < paper_id) = 100;
            highBinarize = zeros(size(Icrop));
            highBinarize(clusteredIcrop > paper_id) = 100;
            
            subImageBw = imbinarize(lowBinarize + highBinarize, 99);
            increased_dilate = (min(squareSizeEstimation)/2)-mod((min(squareSizeEstimation)/2),2) + 1;
            SE = strel('square', increased_dilate);
            subImageBwDilate = imdilate(subImageBw,SE);

            % increase the alloweded circle ratio and area constraints
            % for the region indetification
            estimatedDilatedArea = (round(squareSizeEstimation(1)) + round((increased_dilate))) * (round(squareSizeEstimation(2)) + round((increased_dilate)));
            
            circleRatio = 1 - (0.2 * relax);
            maximumArea = estimatedDilatedArea + ((estimatedDilatedArea * (0.2 * relax)));
            minimumArea = estimatedDilatedArea - ((estimatedDilatedArea * (0.2 * relax)));            
            
            s = regionprops(subImageBwDilate, 'Centroid','Area','Orientation','MajorAxisLength','MinorAxisLength');

            centroids = cat(1, s.Centroid);
            
            if debug
                
                disp("C_sorted");
                disp(C_sorted);
                disp("paper_mask_mean");
                disp(paper_mask_mean);
                disp("paper_mask_std");
                disp(paper_mask_std);
                disp("grey_val");
                disp(grey_val);

                if size(centroids, 1) == 1
                    disp(['Circle Ratio: ' num2str(s.MinorAxisLength/s.MajorAxisLength) ' (' num2str(circleRatio) ')']); 
                    disp(['Area: ' num2str(s.Area) ' (' num2str([minimumArea maximumArea]) ')']); 
                end
            end
            
            % deep debug (a lot of figures)
            if false
                if size(centroids, 1) == 1
                    figure(1)
                    clf;
                    imshow(I)
                    hold on
                    plot(partialPoints(i,1), partialPoints(i,2), 'r*')

                    plot(vertices(1,1), vertices(1,2), 'b*')
                    plot(vertices(2,1), vertices(2,2), 'b*')
                    plot(vertices(3,1), vertices(3,2), 'b*')
                    plot(vertices(4,1), vertices(4,2), 'b*')            

                    rectangle('Position', bbox, 'EdgeColor','g', 'LineWidth', 1);

                    figure(2);
                    clf;
                    imshow(Icrop);
                    title('Icrop');
                    figure(3);
                    clf;
                    imshow(lowBinarize);
                    title('lowBinarize');
                    figure(4);
                    clf;
                    imshow(highBinarize);
                    title('highBinarize');                    
                    figure(5);
                    clf;
                    imshow(subImageBw);
                    title('subImageBw');
                    figure(6);
                    clf;
                    imshow(subImageBwDilate);
                    title('subImageBwDilate');
                    figure(7);
                    clf;
                    imagesc(clusteredIcrop);
                    title('clusteredIcrop');

                    hold on
                    plot(centroids(:,1),centroids(:,2),'ro');

                    t = linspace(0,2*pi,50);
                    a = s.MajorAxisLength/2;
                    b = s.MinorAxisLength/2;
                    Xc = s.Centroid(1);
                    Yc = s.Centroid(2);
                    phi = deg2rad(-s.Orientation);
                    x = Xc + a*cos(t)*cos(phi) - b*sin(t)*sin(phi);
                    y = Yc + a*cos(t)*sin(phi) + b*sin(t)*cos(phi);
                    plot(x,y,'r','Linewidth',1)                        

                    hold off;
                    pause;
                    disp('Debug');
                else
                    disp('Ouch!');
                end
            end

            if(size(centroids, 1) == 1)
                if s.MinorAxisLength/s.MajorAxisLength > circleRatio && ...
                        s.Area <= maximumArea && s.Area >= minimumArea && ...
                        s.MajorAxisLength <= sqrt((2*squareSizeEstimation(1))^2 + (2*squareSizeEstimation(2))^2)
                    detectedPoints = [detectedPoints; centroids + [bbox(1) bbox(2)]];
                else
                    if debug
                        disp('Square candidate rejected');
                    end
                end
            end
        end
    end
    
    detectedPoints = removeDuplicates(detectedPoints, min(squareSizeEstimation));
    wholePoints = [detectedPoints; partialPoints];
    wholePoints = removeDuplicates(wholePoints, min(squareSizeEstimation));
    
	if debug
        figure(1);
        clf;
        imshow(I);
        hold on;
        plot(partialPoints(:,1), partialPoints(:,2), 'rx');
        plot(detectedPoints(:,1),detectedPoints(:,2),'b+');
        plot(wholePoints(:,1), wholePoints(:,2), 'go');
        hold off;
        title('Filling pattern iteration growing');
        pause;
    end
    
    % pattern extents estimation
    min_x = min(wholePoints(:,1) - squareSizeEstimation(1)/2);
    max_x = max(wholePoints(:,1) + squareSizeEstimation(1)/2);
    min_y = min(wholePoints(:,2) - squareSizeEstimation(2)/2);
    max_y = max(wholePoints(:,2) + squareSizeEstimation(2)/2);
    
    BoundingBox = double([min_x min_y (max_x - min_x) (max_y - min_y)]);
end