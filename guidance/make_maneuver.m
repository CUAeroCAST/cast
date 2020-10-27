% This function implements a guidance algorithm to generate a maneuver and timing. This
% function MUST be rate limited such that the estimator operates in real time.
function [maneuver, delay] = make_maneuver(estimate, guidanceParams)
 % probability = calculateProbability(estimate);
 [gradx,grady] = gradient(probability);
 [satellitex,satellitey] = find_sat_position(estimate);
 maneuverx = gradx(satellitex);
 maneuvery = grady(satellitey);
 doSomething = {estimate, guidanceParams};
 maneuver = 0;
 delay = 0;
end