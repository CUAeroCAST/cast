% This function MUST be rate limited such that the estimator operates in real time.
function [maneuver,tAfter,stateAfter] = make_maneuver(propogation,satelliteState,timeToCol,burnTimesteps)
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
[pdf,probability,xrange,yrange] = calculate_probability(propogation);
% If probability is too great, impliment maneuver
 if probability>.25   %Initial probability of a near miss scenerio
     % Calculate the gradient of the pdf to find the maneuver direction
     [gradx,grady] = gradient(pdf);
     [satellitex,satellitey] = find_sat_position(propogation);
     maneuverx = -gradx(satellitex(1));
     maneuvery = -grady(satellitey(1));
     unit_rad = satelliteState(1:3)/norm(satelliteState(1:3));
     unit_along = satelliteState(4:6)/norm(satelliteState(4:6));
     unit_cross = cross(unit_rad,unit_along);
     Q = [unit_rad';
         unit_along';
         unit_cross'];
     burnDirection = Q'*[0;maneuverx;maneuvery];
%      burnDirection = [.0041;.0011;0]; 
     % Salculate the burn time
     [burnTime,tAfter,stateAfter] = find_burn_time(propogation,satelliteState,...
         burnDirection,xrange,yrange,timeToCol,burnTimesteps);
     % Output the maneuver
     maneuver = [maneuverx,maneuvery,burnTime];
 else
     % All 0 maneuver if the probability of collision is low enough
     maneuver = [0,0,0];
     tAfter = 0;
     stateAfter = 0;
 end
end