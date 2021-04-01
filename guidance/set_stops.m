function arduinoParams = set_stops(collisionEstimate, arduinoParams)
 xstop = collisionEstimate.predState(1) + 2*sqrt(collisionEstimate.Ppred(1,1));
 ystop = collisionEstimate.predState(2) + 2*sqrt(collisionEstimate.Ppred(2,2));
 arduinoParams.xStop = xstop;
 arduinoParams.yStop = ystop;
end