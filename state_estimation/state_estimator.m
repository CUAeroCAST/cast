% This function will implement a state estimation algorithm on a set of sensor readings
function [estimate, estimatorParams] = state_estimator(sensorReading, time,...
                                                       estimatorParams)
 %Calculate the requested step size and update current estimator time
 stepSize = time - estimatorParams.currentTime; 
 if(stepSize < 0)
     error("State estimation is receiving request for invalid time")
 end
 estimatorParams.currentTime = time;
 %Update sensor covariance
 
 %Estimate the requested state and correct based on sensor reading
 [estimate.predState, estimate.Ppred] = predict_ekf(estimatorParams.filter, stepSize);
 if(~any(isnan(sensorReading)))
     %Sensor location
     xs = 0;
     ys = 0;
    [estimate.corrState, estimate.Pcorr] = correct_ekf(estimatorParams.filter,sensorReading,estimate,xs,ys);
    %Update filter from correction
    estimatorParams.filter.StateCovariance = estimate.Pcorr;
    estimatorParams.filter.State = estimate.corrState;
 else
    estimate.corrState = NaN(1,4);
    estimate.Pcorr = NaN(4,4);
    %Update from prediction
    estimatorParams.filter.StateCovariance = estimate.Ppred;
    estimatorParams.filter.State = estimate.predState;
 end
 %Update the filter object current time (doesn't occur in desync)
 estimatorParams.currentTime = time;
end
