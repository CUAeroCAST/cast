function [burnTime,tAfter,stateAfter] = find_burn_time(estimate,satelliteState,direction,timeToCol, colState)
estimate.Pcorr = estimate.Ppred;
estimate.corrState = estimate.predState;
% Get the position of the debris
mu = 3.98600e14;

% Calculating the pdf
xmax = estimate.corrState(1)+2*sqrt(estimate.Pcorr(1,1));
xmin =  estimate.corrState(1)-2*sqrt(estimate.Pcorr(1,1));
ymax = estimate.corrState(2)+2*sqrt(estimate.Pcorr(2,2));
ymin =  estimate.corrState(2)-2*sqrt(estimate.Pcorr(2,2));
% Transform the debris position
unitRad = satelliteState(1:3)/norm(satelliteState(1:3));
unitAlong = satelliteState(4:6)/norm(satelliteState(4:6));
unitCross = cross(unitRad,unitAlong);
Q = [unitAlong;unitCross;unitRad]';  % make sure this Q goes the right way

%Transform pdf coordinates into cartesian coordiantes

minvecTemp = Q * [xmin; ymin; 0] + colState;
maxvecTemp = Q * [xmax; ymax; 0] + colState;
maxvec = max([minvecTemp'; maxvecTemp']);
minvec = min([minvecTemp'; maxvecTemp']);

direction = Q *[cosd(direction); sind(direction); 0];

for i = 1:30
 [tAfter,stateAfter] = ode45(@(tAfter,stateAfter)...
            orbit_prop_maneuver(tAfter,stateAfter,mu,direction,i),[0,timeToCol],satelliteState);
 maneuverPos = stateAfter(end, 1:3);
    if all(maxvec > maneuverPos) && all(maneuverPos > minvec)
     continue
    else
     break
    end
end
burnTime = i;

end