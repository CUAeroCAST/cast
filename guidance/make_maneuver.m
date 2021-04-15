% This function MUST be rate limited such that the estimator operates in real time.
function maneuver = make_maneuver(propogation, guidanceParams)
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
     R = guidanceParams.angleOffset;
     maneuverxy = R * [maneuverx; maneuvery];
     maneuverx = maneuverxy(1);
     maneuvery = mamneuverxy(2);
     
     directionIndex = round(atan2d(maneuvery,maneuverx));
     if directionIndex<0
         directionIndex = 360+directionIndex;
     end
     burnTime = find_burn_time(propogation, directionIndex, guidanceParams);

     % Output the maneuver
     maneuver = [maneuverx,maneuvery,burnTime];
 else
     % All 0 maneuver if the probability of collision is low enough
     maneuver = [0,0,0];
 end
end
