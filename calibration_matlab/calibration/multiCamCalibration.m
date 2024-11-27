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

% **Common settings for multiple cameras**
resize_factors = struct('realsense_rgb', 1, 'thermal', 1, 'rgb', 1, 'ir1', 1, 'ir2', 1, 'ir3', 1); % Rescale factor for each camera
gamma_values = struct('realsense_rgb', 1.2, 'thermal', 1.3, 'rgb', 1.2, 'ir1', 1.2, 'ir2', 1.2, 'ir3', 1.2); % Gamma values for each camera

isCirclePattern = false;

columns = 6;
rows = 4;
squareSize = 21;
total_vertices = rows * columns;
centerDistance = squareSize * 2;

worldPoints = generateCheckerboardPoints([(rows + 1), (columns + 1)], centerDistance);

th_cmap = loadCustomCmap(); % Load colormap for thermal images
disp("GENERIC");

resultsFile = [dataPath, '/saved_results.mat']; % Path to save calibration results

% **Manual exclusion files for each camera**
manualExcludedFiles = struct( ...
    'realsense_rgb', [dataPath, '/manual_excluded_realsense_rgb.txt'], ...
    'thermal', [dataPath, '/manual_excluded_thermal.txt'], ...
    'rgb', [dataPath, '/manual_excluded_rgb.txt'], ...
    'ir1', [dataPath, '/manual_excluded_ir1.txt'], ...
    'ir2', [dataPath, '/manual_excluded_ir2.txt'], ...
    'ir3', [dataPath, '/manual_excluded_ir3.txt'] ...
);

% Check if calibration results already exist
if isfile(resultsFile)
    group = 'Updates';
    pref = 'Conversion';
    title = 'Load results';
    quest = {'Saved results found in folder: load?'};
    pbtns = {'Yes', 'No'};
    [loadResults, ~] = uigetpref(group, pref, title, quest, pbtns);
end

if strcmp(loadResults, 'yes')
    load(resultsFile);
    fprintf("<! Loaded results %s>\n", resultsFile);
end

% **Dynamically define datasets for all cameras**
cameraSets = struct(); 
cameraSets.realsense_rgb = imageDatastore([dataPath, '/Realsense_RGB']);
cameraSets.thermal = imageDatastore([dataPath, '/Thermal']); 
cameraSets.rgb = imageDatastore([dataPath, '/RGB']); 
cameraSets.ir1 = imageDatastore([dataPath, '/IR1']); 
cameraSets.ir2 = imageDatastore([dataPath, '/IR2']); 
cameraSets.ir3 = imageDatastore([dataPath, '/IR3']); 

% **Check if all datasets have the same number of files**
assert(all(cellfun(@(x) length(cameraSets.realsense_rgb.Files) == length(x.Files), struct2cell(cameraSets))), ...
    'Datasets must have the same number of files!');

fprintf("<! CALIBRATING !>\n");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% **Extract feature points for each camera**
imgData = struct();
for camType = fieldnames(cameraSets)' 
    camType = camType{1};
    fprintf("<! Processing %s camera !>\n", camType);

    % Call landmarksDetector to extract corner points
    [imgData.(camType)] = landmarksDetector( ...
        cameraSets.(camType), camType, columns, rows, gamma_values.(camType), ...
        false, debug, isCirclePattern, resize_factors.(camType), 3, 0.7 ...
    );

    showResults(cameraSets.(camType), imgData.(camType), 1, 1);

    % Handle manual exclusion files
    manualExcludedFile = manualExcludedFiles.(camType);
    if isfile(manualExcludedFile)
        manual_excluded = str2double(readlines(manualExcludedFile));
        manual_excluded = sort(unique(manual_excluded(~isnan(manual_excluded))'));
    else
        manual_excluded = [];
    end

    fprintf("Loaded %d excluded indices (%s camera)\n", length(manual_excluded), camType);

    if ~isempty(manual_excluded)
        for i = 1:imgData.(camType).numImages
            if ~isempty(find(manual_excluded == i, 1))
                imgData.(camType).excluded(i) = 1;
            end
        end
    end
    imgData.(camType).excludedPipeline = imgData.(camType).excluded; 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% **Compute extrinsics for each camera pair**
calibrationResults = struct();
cameraPairs = nchoosek(fieldnames(cameraSets), 2); % Dynamically generate all camera pairs

for i = 1:size(cameraPairs, 1)
    cam1 = cameraPairs{i, 1};
    cam2 = cameraPairs{i, 2};

    fprintf("<! CALIBRATING %s <-> %s !>\n", cam1, cam2);

    valU = (~imgData.(cam1).excluded & ~imgData.(cam2).excluded)'; % Valid indices
    sys_fValU = find(valU);
    fprintf("\tNumber of valid indices: %d\n", length(sys_fValU));
    imagePoints = [];
    imagePoints(:, :, :, 1) = imgData.(cam1).points(:, :, sys_fValU);
    imagePoints(:, :, :, 2) = imgData.(cam2).points(:, :, sys_fValU);

    % Calibrate extrinsics
    [cameraParams, stereoSysValidImageIndices, stereoSysEstimationErrors] = ...
        estimateCameraParameters(imagePoints, worldPoints, 'NumRadialDistortionCoefficients', 3);

    % Save calibration results
    calibrationResults.(sprintf('%s_%s', cam1, cam2)) = struct( ...
        'cameraParams', cameraParams, ...
        'stereoSysValidImageIndices', stereoSysValidImageIndices, ...
        'stereoSysEstimationErrors', stereoSysEstimationErrors ...
    );

    % Display reprojection errors
    figure('Name', sprintf('%s <-> %s', cam1, cam2));
    clf;
    hRef = showReprojectionErrors(cameraParams);
    hold off;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% **Save multi-camera calibration results**
save(resultsFile, 'imgData', 'calibrationResults', 'cameraSets', 'worldPoints');
fprintf("<! Calibration Results saved to %s !>\n", resultsFile);
