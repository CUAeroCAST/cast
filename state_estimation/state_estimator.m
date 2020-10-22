% This function will implement a state estimation algorithm on a set of sensor readings
function [estimate, estimatorParams] = state_estimator(sensorReading, time,...
                                                       estimatorParams)
 %Calculate the requested step size and update current estimator time
 stepSize = estimatorParams.currentTime - time; 
 estimatorParams.currentTime = time;
 
 %Estimate the requested state and correct based on sensor reading
 estimate.state = predict(estimatorParams.filter, stepSize);
 estimate.correctedState = correct(estimatorParams.filter,sensorReading);
end