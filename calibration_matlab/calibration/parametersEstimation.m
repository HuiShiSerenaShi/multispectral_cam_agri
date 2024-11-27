function [ScameraParams, SestimationErrors, indices_to_be_excluded] = parametersEstimation(visImagePoints, thermImagePoints, worldPoints, debug, indices_to_be_excluded, ids_manual_excluded)
    if nargin > 5
       indices_to_be_excluded = unique([indices_to_be_excluded ids_manual_excluded]);
    end
    
    visImagePoints(:,:,indices_to_be_excluded) = [];
    thermImagePoints(:,:,indices_to_be_excluded) = [];
    
    imagePoints = [];
    imagePoints(:,:,:,1) = visImagePoints;
    imagePoints(:,:,:,2) = thermImagePoints;

    [ScameraParams, SPairUsed, SestimationErrors] = estimateCameraParameters(imagePoints, worldPoints,'NumRadialDistortionCoefficients',3);

    if debug
        displayErrors(SestimationErrors, ScameraParams);       
        figure(57)
        clf;
        showReprojectionErrors(ScameraParams);
    end
end