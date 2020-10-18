% This function will implement a state estimation algorithm on a set of sensor readings
function [estimate, estimatorParams] = state_estimator(sensorReading, time,...
                                                              estimatorParams)
 
 fprintf('state_estimator has not been (fully) implemented\n')
 estimate.state = predict(estimatorParams.filter, estimatorParams.stepSize);
 estimate.correctedState = correct(estimatorParams.filter,sensorReading);
end