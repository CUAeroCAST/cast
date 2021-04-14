% This function MUST be rate limited such that the estimator operates in real time.
function [maneuver, tManeuver, stateManeuver] = make_maneuver_sim(propogation, satState, timeToCol, guidanceParams)
% This function implements a guidance algorithm to generate a maneuver and timing. 
% Calculates the probability of collision and then plans appropriate
% maneuver using the pdf gradient if probability is too high.
% Inputs:
% propogation: the propogated state at colision and its covariance
% Outputs:
% maneuver: the maneuver direction and burn time
% Author: Jason Balke | Project: CAST | Date: 10/29/20
%----------------------------------------------------------------------------------------%
% Calculate the probability of collision
[pdf, probability] = calculate_probability(propogation, guidanceParams);
% If probability is too great, impliment maneuver
 if probability > guidanceParams.threshold   %Initial probability of a near miss scenerio
     % Calculate the gradient of the pdf to find the maneuver direction
     [gradx, grady] = gradient(pdf);
     
     [satellitex, satellitey] = find_sat_position(propogation, guidanceParams);
     maneuverx = -gradx(satellitey, satellitex);
     maneuvery = -grady(satellitey, satellitex);
     
%      unit_rad = satelliteState(1:3) / norm(satelliteState(1:3));
%      unit_along = satelliteState(4:6) / norm(satelliteState(4:6));
%      unit_cross = cross(unit_rad, unit_along);
%      Q = [unit_rad, unit_along, unit_cross];

     direction = atan2d(maneuvery,maneuverx);
     colState = guidanceParams.chiefState(1:3);
     [burnTime, tManeuver, stateManeuver] = find_burn_time(propogation, satState, direction, timeToCol, colState);

     % Output the maneuver
     maneuver = [maneuverx,maneuvery,burnTime];
 else
     % All 0 maneuver if the probability of collision is low enough
     maneuver = [0,0,0];
     tManeuver = 0;
     stateManeuver = 0;
 end
end
