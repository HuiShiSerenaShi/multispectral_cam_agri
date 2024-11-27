function [imgData] = landmarksDetector(imageSet,...
    name,columns,rows,gamma,swapWhBh,debug,isCircle, rescale, thresh, solidity)

total_vertices = rows*columns;

skip_homography = false;
detect_edges = false;

% read one to init stuff and speed up
Itmp = readimage(imageSet,1);
Itmp = im2gray(Itmp);

imgData = struct;
imgData.name = name;
imgData.numImages = length(imageSet.Files);
imgData.imgSize = size(Itmp);
imgData.gamma = gamma;
imgData.swapWhBh = swapWhBh;
imgData.rescale = rescale;
imgData.threshold = thresh;
imgData.solidity = solidity;

i = 1;

imgData.excluded = zeros([imgData.numImages, 1]);
imgData.points = zeros(total_vertices,2,imgData.numImages);
partialExtraction = [];
bBBox = zeros(imgData.numImages, 4);
%indicesToExclude = [];
H = [];

defaultRelax = 1;
relax = defaultRelax;
fprintf([name, '\n']);
fprintf('Progress:             ');

while i <= imgData.numImages
    fprintf('\b\b\b\b\b\b%5.2f%%', i / imgData.numImages * 100);
    %disp([name,' image: ',num2str(i),' file ',imageSet.Files{i}]);
    if relax > 5
        bBBox(i,:) = [0,0,0,0];
        imgData.excluded(i) = 1;
        relax = defaultRelax;
        i = i + 1;
        H = [];
        continue;
    end

    Itmp = readimage(imageSet,i);

    Itmp = fixImage(Itmp, gamma, swapWhBh, rescale);

    
    % Itmp = im2double(im2gray(Itmp));
    % Itmp = (double(Itmp) - double(min(Itmp(:)))) ./ ...
    %     (double(max(Itmp(:))) - double(min(Itmp(:))));
    % Itmp = Itmp.^gamma;

    %Itmp = adapthisteq(Itmp);
    
    % Itmp = imadjust(Itmp);
    if isCircle
        se = strel("sphere",1);
        Itmp = imdilate(Itmp,se);
    end
    % 
    % 
    % SWITCH WHITE HOT <-> BLACK HOT!!
    % if swapWhBh
    %     Itmp = imcomplement(Itmp);
    % end
    if debug
        figure(Name="Debug image TH calib"); hold on;
        imshow(Itmp); hold off;
    end
    
    % if abs(rescale-1) < eps(rescale)*4
    %     Itmp = imresize(Itmp, rescale);
    % end
% binarize
%Itmp(Itmp > 0.45) = 1;

    if imgData.imgSize ~= size(Itmp)
        imgData.imgSize = size(Itmp);
    end

    if  ~exist('H','var') || isempty(H)
        % the threshold is quite crucial to detect not perfect squares,
        % then the bounds to solidity, mean and max area and bigget
        % connected component will remove the noise
        [H,orig_bbBox,partialPoints,squareSizeEstimation] = ...
            patternExtraction(Itmp,columns,rows,thresh,solidity,skip_homography, ...
            detect_edges,debug);

        % save partial images
        if length(partialPoints) < columns * rows
            %disp(['Partial extraction - centroids found: ', ...
            %    num2str(size(partialPoints,1))]);
            partialExtraction(end+1) = i;
        elseif length(partialPoints) > columns*rows
            %disp(['Over extraction - centroids found: ', ...
            %    num2str(size(partialPoints,1))]);
            relax = Inf;
            continue;
        end

        % unable to estimate homography skip to the next one
        if isempty(H)
            relax = Inf;
            continue;
        end
    end

    if length(squareSizeEstimation) == 1 || isempty(partialPoints)
        relax = Inf;
        continue;
    end

    UnPerspectI = imwarp(Itmp,H,'OutputView', imref2d(size(Itmp)));
    UnPerspectI(isnan(UnPerspectI)) = 0;

    % Instead of filling the missing pattern, the function is called anyway
    % in order to refine the estimation of the centroid.
    % if size(partialPoints, 1) < total_vertices
    if size(partialPoints,1) > 0
        while 1
            [wholePoints,bbBox] = fillPattern(UnPerspectI,partialPoints, ...
                squareSizeEstimation,relax,columns,rows,debug);

            % grows the search to the new neighborhood
            if size(wholePoints, 1) < total_vertices && ...
                    size(partialPoints,1) < size(wholePoints,1)
                partialPoints = wholePoints;
            else
                break;
            end
        end
    else
        wholePoints = partialPoints;
        bbBox = orig_bbBox;
    end

    [relax,skip,gridPoints] = buildGrid(Itmp,UnPerspectI,bbBox, ...
        orig_bbBox,rows,columns,H,relax,total_vertices,wholePoints,debug);
    if(skip)
        continue;
    else
        imgData.points(:,:,i) = gridPoints;
        bBBox(i,:) = bbBox;
    end
    i = i + 1;
    relax = defaultRelax;
    H = [];
end
imgData.excludedPipeline = imgData.excluded;
end