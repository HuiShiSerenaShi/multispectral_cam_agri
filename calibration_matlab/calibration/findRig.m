function [H,centroids,meanDiameter] = findRig(im,columns,rows, ...
    threshold,solidity,skip_homography,debug)
    % Extract features
    % The minimum area is still needed?
    max_area = (size(im,1)*size(im,2))/(((2*columns) - 1) * ((2*rows) - 1));
    [regions,mserCC] = detectMSERFeatures(im,'ThresholdDelta', threshold,'MaxAreaVariation', 0.9,'RegionAreaRange', [round(max_area/48), round(max_area)]);
    stats = regionprops('table',mserCC,'Centroid','EquivDiameter','MajorAxisLength','MinorAxisLength','FilledArea','Solidity');

    % extract circle like regions
    eccentricityIdx = ...
        stats.FilledArea <= max_area & ... 
        stats.FilledArea <= mean(stats.FilledArea) + (2*std(stats.FilledArea)) & ... 
        stats.Solidity > solidity & ... 
        stats.MinorAxisLength./stats.MajorAxisLength > 0.2;
    
    % extract mean diameter and circular regions
    meanDiameter = mean(stats.EquivDiameter(eccentricityIdx));
    circularRegions = regions(eccentricityIdx);
    
    if isempty(circularRegions)
       centroids = [];
       H = [];
       return;
    end

    % extract the bigger connected component    
    pp = [];
    if iscell(circularRegions.PixelList)
        for i=1:length(circularRegions.PixelList)
           pp = [pp; circularRegions.PixelList{i,1}(:,1) circularRegions.PixelList{i,1}(:,2)];
        end
    else
        pp = circularRegions.PixelList{1,1};
    end
    
    gradMask = zeros(size(im));
    gradMask(sub2ind(size(gradMask), pp(:,2), pp(:,1))) = 255;
    strel_size = meanDiameter - mod(meanDiameter,2) + 3;
    SE = strel('square', strel_size);
    gradMaskDilate = imdilate(gradMask,SE);
    cc = bwconncomp(gradMaskDilate);
    [~, max_index] = max(cellfun('size', cc.PixelIdxList, 1));
    
    % filter the pixel list and the regions centroid to be within the biggest cc    
    % the regions pixel is no longer used instead of them it's better to
    % use the regions centroid
    %[~,ia,~] = intersect(sub2ind(size(gradMaskDilate), pp(:,2), pp(:,1)), cc.PixelIdxList{max_index});
    %pp_final = pp(ia,:);
    
    centroids = [];
    if iscell(circularRegions.PixelList)
        for i=1:length(circularRegions.Centroid)
            if sum(ismember(cc.PixelIdxList{max_index}, ...
                sub2ind(size(gradMaskDilate), floor(circularRegions.Centroid(i,2)), floor(circularRegions.Centroid(i,1))))) > 0
                centroids(end+1,:) = circularRegions.Centroid(i,:);
            end
        end
    else
        centroids(end+1,:) = circularRegions.Centroid(1,:);
    end
    
    if debug
        % % Print purpose
        % figure(1);
        % clf;
        % imshow(im);
        % hold on;
        % plot(regions,'showPixelList',true, 'showEllipses', true);
        % print(gcf(),'mser','-depsc');
        
        % clf;
        % imshow(im);
        % hold on;
        % plot(circularRegions,'showPixelList',true, 'showEllipses', true);
        % print(gcf(),'mser_filt','-depsc');
        
        % clf;
        % imshow(gradMaskDilate);
        % print(gcf(),'mser_filt_dilated','-depsc');
        
        % clf;
        % grandMaskDilateFiltered = zeros(size(im));
        % grandMaskDilateFiltered(cc.PixelIdxList{max_index}) = 255;
        % imshow(grandMaskDilateFiltered);
        % hold on;
        % plot(centroids(:,1),centroids(:,2),'r*');
        % print(gcf(),'mser_filt_dilated_cc','-depsc');        
        
        figure(1);
        clf;        
        subplot(2,2,1);
        imshow(im);
        hold on;
        plot(regions,'showPixelList',true, 'showEllipses', true);
        title('MSER');

        subplot(2,2,2);
        imshow(im);
        hold on;
        plot(circularRegions,'showPixelList',true, 'showEllipses', true);
        title('MSER filtered');
        
        subplot(2,2,3);
        imshow(gradMaskDilate);
        title('Binary regions');

        subplot(2,2,4);
        grandMaskDilateFiltered = zeros(size(im));
        grandMaskDilateFiltered(cc.PixelIdxList{max_index}) = 255;
        imshow(grandMaskDilateFiltered);
        hold on;
        plot(centroids(:,1),centroids(:,2),'r*');
        title('Target connected component');
    end
    
    if skip_homography
        H = [];
    else
        % To fit the rectangle we use only the filtered centroids so we
        % easily reject obtains smooth edges around the connected component
        H = fitRectangle(im, centroids, debug);
    end
end