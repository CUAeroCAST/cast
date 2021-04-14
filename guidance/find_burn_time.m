function [burnTime,tAfter,stateAfter] = find_burn_time(estimate,satelliteState,direction,timeToCol, colState)
estimate.Pcorr = estimate.Ppred;
estimate.corrState = estimate.predState;
% Get the position of the debris
mu = 3.98600e14;

% Calculating the pdf
xmax = estimate.corrState(1)+2*sqrt(estimate.Pcorr(1,1));
xmin =  estimate.corrState(1)-2*sqrt(estimate.Pcorr(1,1));
ymax = estimate.corrState(3)+2*sqrt(estimate.Pcorr(3,3));
ymin =  estimate.corrState(3)-2*sqrt(estimate.Pcorr(3,3));
% Transform the debris position
unitRad = satelliteState(1:3)/norm(satelliteState(1:3));
unitAlong = satelliteState(4:6)/norm(satelliteState(4:6));
unitCross = cross(unitRad,unitAlong);
Q = [unitRad;unitAlong;unitCross]';  % make sure this Q goes the right way

%Transform pdf coordinates into cartesian coordiantes

minvec = Q * [xmin; ymin; 0] + colState;
maxvec = Q * [xmax; ymax; 0] + colState;

direction = Q *[cosd(direction); sind(direction); 0];

for i = 1:30
 [tAfter,stateAfter] = ode45(@(tAfter,stateAfter)...
            orbit_prop_maneuver(tAfter,stateAfter,mu,direction,i),[0,timeToCol],satelliteState);
 maneuverPos = stateAfter(end, 1:3);
    if any(maxvec > maneuverPos') && any(maneuverPos' > minvec)
     continue
    else
     break
    end
end
burnTime = i;

end