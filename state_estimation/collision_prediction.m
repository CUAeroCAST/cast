function collisionEstimate = collision_prediction(estimate, estimatorParams,collisionEstimate)
% This function forward predicts the current estimate until we reach point
% of collision and returns the estimate of the object's state at this point
 if(~any(isnan(estimate.corrState), "all") && estimate.corrState(3)<0 && estimate.corrState(1) > 0)
  collisionEstimate.collisionTime = -estimate.corrState(1)/estimate.corrState(3) + ...
                  estimatorParams.currentTime;
  %need to calculate it when we can avoid
  collisionEstimate = desync_predict(collisionEstimate.collisionTime, estimatorParams);   
 elseif(estimate.predState(3)<0 && estimate.predState(1) > 0)
  collisionEstimate.collisionTime = -estimate.predState(1)/estimate.predState(3) + ...
                  estimatorParams.currentTime;
  %need to calculate it when we can avoid
  collisionEstimate = desync_predict(collisionEstimate.collisionTime, estimatorParams); 
 else
  %do nothing
 end
end

