%This function predicts without updating the parent scenario in order to
%get covariance at time of collision.
function [estimate] = desync_predict(time,estimatorParams)
 %Calculate the requested step size and update current estimator time
 stepSize = time - estimatorParams.currentTime; 
 if(stepSize < 0)
     error("State estimation is receiving request for invalid time")
 end
 copyFilter = clone(estimatorParams.filter);
 copyFilter.MotionModel = 'Custom';
 %Process noise model used in matlab for 2D constant velocity
 copyFilter.ProcessNoise = [(stepSize^3)/3, (stepSize^2)/2,0,0;
                            (stepSize^2)/2, stepSize,0,0;
                            0,0,(stepSize^3)/3, (stepSize^2)/2;
                            0,0,(stepSize^2)/2, stepSize];
 copyFilter.ProcessNoise = copyFilter.ProcessNoise./estimatorParams.qGain;
 copyFilter.StateTransitionModel = [1,stepSize,0,0;
                                    0,1,0,0;
                                    0,0,1,stepSize;
                                    0,0,0,1];
 
 %Estimate the requested state and correct based on sensor reading
%  m = [0;0;0;0];
 [estimate.predState, estimate.Ppred] = predict(copyFilter);
 estimate.collisionTime = time;
end