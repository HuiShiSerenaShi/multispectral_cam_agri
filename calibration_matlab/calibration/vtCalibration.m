%%%%%%%%%%
% DO NOT USE MATLAB R2022b! BROKEN! (changed rigid3d convention from
% premult to postmult broke cameraCalibration
%%%%%%%%%%
%% Clean and prepare Data

close all;
clearvars;
clc;

debug = false;
loadResults = 'no';
% folder containing the current dataset
dataPath = uigetdir;

resize_th = 1;
resize_vis = 1;
vis_gamma = 1.2;
therm_gamma = 1.3; % 1.8 worse

isCirclePattern = false;

columns = 6;
rows = 4;
% Gamma correction factor
%therm_gamma = 1.2;
% Building world points
squareSize = 21;
total_vertices = rows*columns;
centerDistance = squareSize*2;

worldPoints = generateCheckerboardPoints([(rows+1),(columns+1)], ...
centerDistance);

th_cmap = loadCustomCmap();
% default values for image size: rewritten on first image loading

disp("GENERIC");
expPrefix = 'generic';
rotTh = "NO";
rotV = "NO";


resultsFile = [dataPath, '/saved_results.mat'];

% first pruning indices for badly recognized centroids
manualExcludedThFile= [dataPath,'/manual_excluded_th.txt'];
manualExcludedVisFile= [dataPath,'/manual_excluded_vis.txt'];

if isfile(resultsFile)
    group = 'Updates';
    pref =  'Conversion';
    title = 'Load results';
    quest = {'Saved results found in folder: load?'};
    pbtns = {'Yes','No'};
    [loadResults, ~] = uigetpref(group,pref,title,quest,pbtns);
end

if strcmp(loadResults, 'yes')
    load(resultsFile);
    fprintf("<! Loaded results %s>\n", resultsFile);
end

% Thermal
thermImgSet = imageDatastore([dataPath,'/Thermal']);
resizeSet = false;
whiteHot2BlackHot = false;

% Visible
visImgSet = imageDatastore([dataPath,'/Visible']);
assert((length(thermImgSet.Files)==length(visImgSet.Files)),...
    'Datasets must have the same number of files!');
fprintf("<! CALIBRATING !>\n");
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Visible point extraction
[visImgData] = landmarksDetector(...
    visImgSet,'visible',columns,rows,vis_gamma,false,false,isCirclePattern,resize_vis,3,0.7);
vis_img_size = visImgData.imgSize(1:2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('\n');

%% Show visible results

showResults(visImgSet,visImgData,1,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Thermal point extraction
[thermImgData] = landmarksDetector(...
    thermImgSet,'thermal',columns,rows,therm_gamma,whiteHot2BlackHot,debug,isCirclePattern,resize_th,3,0.7);
th_img_size = thermImgData.imgSize(1:2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('\n');

%% Show thermal results

showResults(thermImgSet,thermImgData,1,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Add manually excluded indices for visible camera

if isfile(manualExcludedVisFile)
    manual_excluded_vis = str2double(readlines(manualExcludedVisFile));
    manual_excluded_vis = sort(unique(manual_excluded_vis(~isnan(manual_excluded_vis))'));
else
    manual_excluded_vis = [];
end
manual_excluded_vis = sort(manual_excluded_vis);
fprintf("Loaded %d therm excluded indices\n", length(manual_excluded_vis));

if ~isempty(manual_excluded_vis)
    for i=1:visImgData.numImages
        if ~isempty(find(manual_excluded_vis == i, 1))
            visImgData.excluded(i) = 1;
        end
    end
end
visImgData.excludedPipeline = visImgData.excluded;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Add manually excluded indices for thermal camera

if isfile(manualExcludedThFile)
    manual_excluded_th = str2double(readlines(manualExcludedThFile));
    manual_excluded_th = sort(unique(manual_excluded_th(~isnan(manual_excluded_th))'));
else
    manual_excluded_th = [];
end
manual_excluded_th = sort(manual_excluded_th);
fprintf("Loaded %d therm excluded indices\n", length(manual_excluded_th));

if ~isempty(manual_excluded_th)
    for i=1:thermImgData.numImages
        if ~isempty(find(manual_excluded_th == i, 1))
            thermImgData.excluded(i) = 1;
        end
    end
end
thermImgData.excludedPipeline = thermImgData.excluded;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% stereo system Calibration
    
valU = (~visImgData.excluded & ~thermImgData.excluded)';
sys_fValU = find(valU);
fprintf("\tNumber of valid indices: %d\n", length(sys_fValU));
imagePoints = [];
imagePoints(:,:,:,1) = visImgData.points(:,:,sys_fValU);
imagePoints(:,:,:,2) = thermImgData.points(:,:,sys_fValU);

[ScameraParams, stereoSysValidImageIndices, stereoSysEstimationErrors] = ...
    estimateCameraParameters(imagePoints, worldPoints,'NumRadialDistortionCoefficients',3);

notViA = ~stereoSysValidImageIndices;

% remove automatic not valid indices from pool (indexing only)
for i=1:length(notViA)
    if notViA(i)
        visImgData.excluded(sys_fValU(i)) = 1;
        thermImgData.excluded(sys_fValU(i)) = 1;
        valU(sys_fValU(i)) = 0;
    end
end

figure('Name',[visImgData.name, ' <-> ', thermImgData.name]);
clf;
hRef = showReprojectionErrors(ScameraParams);
hRef.XTick = 1:length(stereoSysValidImageIndices);
hRef.XTickLabel = num2str(find(valU)');
hold off;
figure;
hold on;
showExtrinsics(ScameraParams);
hold off;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
errCutoff = 3;
if errCutoff > 0
    for i=1:length(sys_fValU)
        err_l = mean(sqrt(ScameraParams.CameraParameters1.ReprojectionErrors(:,1,i).^2 + ScameraParams.CameraParameters1.ReprojectionErrors(:,2,i).^2));
        err_r = mean(sqrt(ScameraParams.CameraParameters2.ReprojectionErrors(:,1,i).^2 + ScameraParams.CameraParameters2.ReprojectionErrors(:,2,i).^2));
        if (err_l > errCutoff || err_r > errCutoff)
            visImgData.excluded(sys_fValU(i)) = 1;
            thermImgData.excluded(sys_fValU(i)) = 1;
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
visImgData.excluded = visImgData.excludedPipeline;
thermImgData.excluded = thermImgData.excludedPipeline;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% VIS <-> th homography

valU = (~thermImgData.excluded & ~visImgData.excluded)';

thDistortedPoints = thermImgData.points(:,:,valU);
visDistortedPoints = visImgData.points(:,:,valU);

if ~all((size(thDistortedPoints) == size(visDistortedPoints)))
    disp("FUCK");
    return
else
    matching = size(thDistortedPoints,3);
    fprintf("Found %d matching patterns between L and TH\n", matching);
end

nump = size(thDistortedPoints,1);

thDist = zeros([nump * matching, 2]);
thPoints = zeros([nump * matching, 2]);

visDist = zeros([nump * matching, 2]);
visCamPoints = zeros([nump * matching, 2]);

for i=1:matching
    visDist((1:nump) + (i-1)*nump, :) = visDistortedPoints(:,:,i);
    visCamRect = undistortPoints(visDistortedPoints(:,:,i), ScameraParams.CameraParameters1);
    visCamPoints((1:nump) + (i-1)*nump, :) = visCamRect; % - refCamParams.PrincipalPoint;

    thDist((1:nump) + (i-1)*nump, :) = thDistortedPoints(:,:,i);
    thRect = undistortPoints(thDistortedPoints(:,:,i), ScameraParams.CameraParameters2);
    thPoints((1:nump) + (i-1)*nump, :) = thRect; % - thCamParams.PrincipalPoint;
end

v_H_tform_th = fitgeotform2d(visCamPoints,thPoints,'projective');
th_H_tform_v = fitgeotform2d(thPoints,visCamPoints,'projective');

v_H_th = v_H_tform_th.A;
th_H_v = th_H_tform_v.A;

% plot cameras
v_T_v = rigidtform3d(eye(3), [0 0 0]);
v_T_th = ScameraParams.PoseCamera2;


% TEST
tmp = v_H_th * [visCamPoints, ones([size(visCamPoints,1), 1])]';
H_err_v = (thPoints) - ([tmp(1,:) ./ tmp(3,:); tmp(2,:) ./ tmp(3,:)]');
tmp = H_err_v.^2;
dist_v = sqrt(tmp(:,1) + tmp(:,2));
E_H_err_v = mean(dist_v);
disp(["Mean reproj distance (px): ", num2str(E_H_err_v)]);
std_H_err_v = std(dist_v);
disp(["Sigma reproj distance (px): ", num2str(std_H_err_v)]);
clear tmp;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Compute remaining params & view
close all;
figure(Name="Left<->Thermal Superposition");
hold on;
fprintf('Image List\n');
fprintf('Progress:\n');
s = 1; 
e = size(thermImgSet.Files,1);

%[K_therm, ~] = cameraIntrinsicsToOpenCV(thCamParams);

vis_new_orig = [0,0];

th_new_orig = [0,0];

for imgn=s:e
    if thermImgData.excluded(imgn) || visImgData.excluded(imgn)
        continue;
    end
    fprintf('%d/%d\n',imgn,e);
    th_image = readimage(thermImgSet,imgn);
    vis_image = readimage(visImgSet,imgn);

    %[Jvis,Jth] = rectifyStereoImages(vis_image,th_image,tform1,tform2);
    
    [Jvis,vis_new_orig] = undistortImage(...
        vis_image,...
        ScameraParams.CameraParameters1,...
        'cubic',...
        OutputView='full');

    [Jth,th_new_orig] = undistortImage(...
        th_image,...
        ScameraParams.CameraParameters2,...
        'cubic',...
        OutputView='full');

    alpha = 0.4;
    
    JHv = mapThermalData(vis_image,Jth,v_H_th,vis_new_orig,th_new_orig);
    JHv = (double(JHv) - double(min(th_image(:)))) ./ (double(max(th_image(:))) - double(min(th_image(:))));
    th_RGB3v = uint8(ind2rgb(im2uint8(JHv),th_cmap));
    
    V = (1-alpha)*im2double(vis_image) + alpha*im2double(th_RGB3v);
    
    montage({V}, BackgroundColor="g", Interpolation="bilinear");
    imwrite(V, [dataPath,'/Superpos/sup',num2str(imgn),'.jpg'],'jpg');
end
disp("Series terminated!");
hold off;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Save results
f = [dataPath,'/saved_results.mat'];
save(f ...
    , "vis_img_size", "th_img_size"...
    , "worldPoints", "world3DPoints"...
    , "visImgData", "thermImgData"...
    , "v_H_th", "stereoSysEstimationErrors" ...
    , "thCamParams", "visCamParams", "ScameraParams");