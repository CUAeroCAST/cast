% This function initializes the estimator, may be a noop for some estimators.
function [offset, estimatorParams] = init_estimator(sensorReadings, estimatorParams)
%Index of any rows without NaNs
idx = find(~any(isnan(sensorReadings),2)); 
if(estimatorParams.llsSeeding)
  batchReadings = sensorReadings(idx(1:estimatorParams.batchSamples),:);
  offset = idx(estimatorParams.batchSamples)+1;

  % Batch LLS seeding
  %Measurement matrix H, takes state x, to measurement vector, y
  H = [1,0,0,0,0,0;0,0,1,0,0,0;0,0,0,0,1,0];
  [r,c] = size(batchReadings);
  %Reshape measurement vector to have dimensions 3*Nx1
  y = reshape(batchReadings', [r*c,1]);
  %Make H block matrix with H tiled down the number of measurements
  H_blc = repmat(H,[r,1]);
  %Use batch LLS to estimate the initial state
  x_LS = H_blc\y;
  %Set initial state to batch LLS output
  estimatorParams.initState = x_LS;
 else
  estimatorParams.initState = sensorReadings(idx(1),:);
  offset = idx(1) + 1;
end
 
 %Initialize Kalman Filter
 estimatorParams.filter=trackingKF('MotionModel','3D Constant Velocity',...
                                   'State', estimatorParams.initState);
end