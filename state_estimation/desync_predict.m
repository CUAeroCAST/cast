%This function predicts without updating the parent scenario in order to
%get covariance at time of collision.
function [estimate] = desync_predict(time,estimatorParams)
 %Calculate the requested step size and update current estimator time
 filter = estimatorParams.filter;
 stepSize = time - estimatorParams.currentTime; 
 if(stepSize < 0)
     error("State estimation is receiving request for invalid time")
 end
 
 [estimate.predState, estimate.Ppred] = predict_ekf(filter, stepSize);
 estimate.collisionTime = time;
end