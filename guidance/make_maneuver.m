% This function implements a guidance algorithm to generate a maneuver and timing. This
% function MUST be rate limited such that the estimator operates in real time.
function [maneuver, delay] = make_maneuver(estimate, guidanceParams)
 [pdf,probability,pError] = calculateProbability(estimate);
 if probability>.001
     [gradx,grady] = gradient(pdf);
     [satellitex,satellitey] = find_sat_position(estimate);
     maneuverx = gradx(satellitex);
     maneuvery = grady(satellitey);
     maneuver = 0;
     delay = 0;
 else
     maneuver = 0;
     delay = 0;
 end
end