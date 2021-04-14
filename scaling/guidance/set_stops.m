function arduinoParams = set_stops(collisionEstimate, arduinoParams, guidanceParams)
 extra = guidanceParams.rDodge;
 xstop = collisionEstimate.predState(1) + 2*sqrt(collisionEstimate.Ppred(1,1)) + extra;
 ystop = collisionEstimate.predState(2) + 2*sqrt(collisionEstimate.Ppred(2,2)) + extra;
 arduinoParams.xStop = min(xstop, arduinoParams.xStop);
 arduinoParams.yStop = min(ystop, arduinoParams.yStop);
end