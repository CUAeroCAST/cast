% This function initializes the estimator, may be a noop for some estimators.
function [offset, estimatorParams] = init_estimator(initialReadings, estimatorParams)
 fprintf('offset is not implemented\n');
 offset = 1;

 % Batch LLS seeding
 H = [1,0,0,0,0,0;0,0,1,0,0,0;0,0,0,0,1,0];
 [r,c] = size(initialReadings);
 y = reshape(initialReadings, [r*c,1]);
 H_blc = repmat(H,[r,1]);
 x_LS = H_blc\y;
 estimatorParams.initState = x_LS;
 
 %Initialize Kalman Filter
 estimatorParams.filter=trackingKF('MotionModel','3D Constant Velocity',...
                                   'State', estimatorParams.initState);
end