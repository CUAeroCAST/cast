%This function predicts without updating the parent scenario in order to
%get covariance at time of collision.
function [estimate] = desync_predict(time,estimatorParams)
 %Calculate the requested step size and update current estimator time
 stepSize = time - estimatorParams.currentTime; 
 if(stepSize < 0)
     error("State estimation is receiving request for invalid time")
 end
 copyFilter = clone(estimatorParams.filter);
 
 %Estimate the requested state and correct based on sensor reading
%  m = [0;0;0;0];
 [estimate.predState, estimate.Ppred] = predict(copyFilter, stepSize);
 estimate.collisionTime = time;
end