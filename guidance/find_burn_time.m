function burnTime = find_burn_time(estimate, directionIndex, guidanceParams)
% Determines the burn time. 
% Maps the pdf onto the collision plane then uses plans a maneuver to move
% the satellite outside the pdf
% Inputs:
% propogation: the propogated state at colision and its covariance
% chiefState: state of the satellite
% direction: x and y direction of the maneuver in the collision plane
% Outputs:
% burnTime: time of the burn
% Author: Jason Balke | Project: CAST | Date: 10/29/20
%----------------------------------------------------------------------------------------%
% https://www.youtube.com/watch?v=VDeZyRtPJvI&ab_channel=AnatollD. :p
global positionTable;

estimate.Pcorr = estimate.Ppred;
estimate.corrState = estimate.predState;

scaling = guidanceParams.scaling;
estimate.corrState = estimate.corrState * scaling;
estimate.Pcorr = estimate.Pcorr * scaling^2;
% Get the position of the collision
colState = guidanceParams.colState;
% Convert to orbital scale
%5 Oct 2020 00:26:51.325 434.915387 0.000000 12.490366 -14.481090 -0.000000 -0.832453 
satelliteState = guidanceParams.chiefState;
unit_rad = satelliteState(1:3) / norm(satelliteState(1:3));
unit_along = satelliteState(4:6) / norm(satelliteState(4:6));
unit_cross = cross(unit_rad, unit_along);
Q = [unit_rad, unit_along, unit_cross];

% Calculating the pdf
xmax = estimate.corrState(1) + 2*sqrt(estimate.Pcorr(1, 1));
xmin =  estimate.corrState(1) - 2*sqrt(estimate.Pcorr(1, 1));
ymax = estimate.corrState(2) + 2*sqrt(estimate.Pcorr(2, 2));
ymin =  estimate.corrState(2) - 2*sqrt(estimate.Pcorr(2, 2));
maxvecCart = Q*[xmax; ymax; 0] + colState(1:3)';
minvecCart = Q*[xmin; ymin; 0] + colState(1:3)';

%Transform pdf coordinates into cartesian coordiantes

burnTime = 30;
miss = true;
while miss && burnTime>0
    maneuverPos = positionTable{burnTime,directionIndex};
    if all(maxvecCart < maneuverPos') || all(maneuverPos' < minvecCart)
        burnTime = burnTime - 1;
    else
     burnTime = min(burnTime+1, 30);
     break
    end
end
end
