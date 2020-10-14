% This function will implement a state estimation algorithm on a set of sensor readings
function [estimate, estimatorParams] = state_estimator(sensorReading, time,...
                                                              estimatorParams)
 fprintf('state_estimator has not been implemented\n')
 doSomething = {sensorReading, time, estimatorParams};
 estimate = struct;
  estimate.state = 0;
  estimate.covariance = 0;
 estimatorParams = doSomething{3};
end