% This function initializes the estimator, may be a noop for some estimators.
function [offset, estimatorParams] = init_estimator(sensorReadings, estimatorParams)
 fprintf('offset is not implemented\n');
 offset = 1;
 estimatorParams.filter=trackingKF('MotionModel','3D Constant Velocity',...
                                   'State', estimatorParams.initState);
end