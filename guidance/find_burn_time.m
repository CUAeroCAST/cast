function [burnTime,tAfter,stateAfter] = find_burn_time(estimate,satelliteState,direction,xrange,yrange,timeToCol,burnTimesteps)
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
estimate.Pcorr = estimate.Ppred;
estimate.corrState = estimate.predState;
% Get the position of the debris
relPositionHill = [estimate.corrState(1) estimate.corrState(3) 0]';
% Convert to orbital scale
%5 Oct 2020 00:26:51.325 434.915387 0.000000 12.490366 -14.481090 -0.000000 -0.832453 
maxVel = .5;
maxDist = 1.5;
maxOrbitVel = 14.50512e3;
maxOrbitDist = 434.915387;
scalingFactorVel = maxOrbitVel/maxVel;
scalingFactorDist = maxOrbitDist/maxDist;
estimate.corrState(1) = estimate.corrState(1)*scalingFactorDist;
estimate.corrState(3) = estimate.corrState(3)*scalingFactorDist;
estimate.corrState(2) = estimate.corrState(2)*scalingFactorVel;
estimate.corrState(4) = estimate.corrState(4)*scalingFactorVel;
estimate.Pcorr(1,1) = estimate.Pcorr(1,1)*scalingFactorDist^2;
estimate.Pcorr(3,3) = estimate.Pcorr(3,3)*scalingFactorDist^2;
estimate.Pcorr(2,2) = estimate.Pcorr(2,2)*scalingFactorVel^2;
estimate.Pcorr(4,4) = estimate.Pcorr(4,4)*scalingFactorVel^2;
% Calculating the pdf
mu = [estimate.corrState(1) estimate.corrState(3)];
sigma = [estimate.Pcorr(1,1) estimate.Pcorr(1,3);estimate.Pcorr(1,3) estimate.Pcorr(3,3)];
[x,y] = meshgrid(xrange,yrange);
X = [x(:) y(:)];
pdf = mvnpdf(X,mu,sigma);
pdf = reshape(pdf,length(x),length(y));
% Transform the debris position
unitRad = satelliteState(1:3)/norm(satelliteState(1:3));
unitAlong = satelliteState(4:6)/norm(satelliteState(4:6));
unitCross = cross(unitRad,unitAlong);
Q = [unitRad';unitAlong';unitCross']';  % make sure this Q goes the right way
relPositionCart = Q'*relPositionHill;
cartState = satelliteState(1:3)+relPositionCart;
%Transform pdf coordinates into cartesian coordiantes
xcart = zeros(3,length(xrange));
ycart = zeros(3,length(yrange));
for i = 1:length(xrange)
   xvec = [xrange(i);0;0];
   xcart(:,i) = Q*xvec;
   xcart(:,i) = xcart(:,i)+satelliteState(1:3); %do more to fix dimensions
end
for i = 1:length(yrange)
    yvec = [0;yrange(i);0];
    ycart(:,i) = Q*yvec;
    ycart(:,i) = ycart(:,i)+satelliteState(1:3);
end
burnTime = 30;
miss = true;
while miss && burnTime>0
    [maneuverPos,tAfter,stateAfter] = find_maneuver_position(satelliteState,burnTime,...
        direction,30,burnTimesteps);
    maneuuverPosRel = maneuverPos-satelliteState(1:3);
    maneuverPosHill = Q'*maneuuverPosRel';
    if maneuverPosHill(1)>min(xrange) || maneuverPos(1)<max(xrange)
        if maneuverPosHill(2)>min(yrange) || maneuverPosHill(2)<max(yrange)
            xind = find(xcart==maneuverPos(1));
            yind = find(ycart==maneuverPos(2));
            miss = false;
            burnTime = burnTime+.01;
            break
        else
            miss = true;
        end
    else
        miss = true;
    end
    burnTime = burnTime-.01;
end
end