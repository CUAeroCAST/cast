% This function initializes the estimator, may be a noop for some estimators.
function [offset, estimatorParams] = init_estimator(sensorReadings, estimatorParams)
%Index of any rows without NaNs
idx = find(~any(isnan(sensorReadings),2));
A = [1,0; 0,0; 0,1; 0,0]; %takes xy measurement to xy state
if(estimatorParams.llsSeeding && (estimatorParams.batchSamples > 0))
  batchReadings = sensorReadings(idx(1:estimatorParams.batchSamples),:);
  offset = idx(estimatorParams.batchSamples)+1;

  % Batch LLS seeding
  %Measurement matrix H, takes state x, to measurement vector, y
  H = eye(2);
  [r,c] = size(batchReadings);
  %Reshape measurement vector to have dimensions 3*Nx1
  y = reshape(batchReadings', [r*c,1]);
  %Make H block matrix with H tiled down the number of measurements
  H_blc = repmat(H,[r,1]);
  %Use batch LLS to estimate the initial state
  x_LS = H_blc\y;
  %Set initial state to batch LLS output
  estimatorParams.initState = A*x_LS;
else
  estimatorParams.initState = A*sensorReadings(idx(1),:)';
  offset = idx(1) + 1;
end

 %Initialize Kalman Filter
 kfOpts = {'MotionModel','2D Constant Velocity',...
           'State', estimatorParams.initState,...
           'MeasurementNoise',estimatorParams.sensorCovariance};
 %2D constant velocity
 estimatorParams.filter.STM = @(dt)[1,dt,0,0;0,1,0,0;0,0,1,dt;0,0,0,1];
 %XY measurement matrix
 estimatorParams.filter.MeasurementModel = [1,0,0,0;0,0,1,0];
 %Initial state
 estimatorParams.filter.State = estimatorParams.initState;
 %Measurement noise
 estimatorParams.filter.MeasurementNoise = estimatorParams.sensorCovariance;
 
 estimatorParams.filter.ProcessNoise = @(dt)[(dt^3)/3, (dt^2)/2,0,0;
                                            (dt^2)/2, dt,0,0;
                                            0,0,(dt^3)/3, (dt^2)/2;
                                            0,0,(dt^2)/2, dt];
estimatorParams.filter.ProcessNoise = @(dt) estimatorParams.qGain*...
                                      estimatorParams.filter.ProcessNoise(dt);
estimatorParams.filter.StateCovariance = eye(4);
end