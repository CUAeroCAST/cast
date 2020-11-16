% This function will implement a state estimation algorithm on a set of sensor readings
function [estimate, estimatorParams] = state_estimator(sensorReading, time,...
                                                       estimatorParams)
 %Calculate the requested step size and update current estimator time
 stepSize = time - estimatorParams.currentTime; 
 if(stepSize < 0)
     error("State estimation is receiving request for invalid time")
 end
 estimatorParams.currentTime = time;
 
 %Estimate the requested state and correct based on sensor reading
 [estimate.predState, estimate.Ppred] = predict(estimatorParams.filter, stepSize);
 if(~any(isnan(sensorReading)))
    [estimate.corrState, estimate.Pcorr] = correct(estimatorParams.filter,sensorReading);
 else
    estimate.corrState = NaN(1,6);
    estimate.Pcorr = NaN(6,6);
 end

end
