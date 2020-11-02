function [burnTime,tAfter,stateAfter] = find_burn_time(estimate,satelliteState,direction,xrange,yrange,timeToCol)
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
% Get the position of the debris
relPositionHill = [estimate.corrState(1) estimate.corrState(3) estimate.corrState(5)]';
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
Q = [unitRad;unitAlong;unitCross]';  % make sure this Q goes the right way
relPositionCart = Q*relPositionHill;
cartState = satelliteState(1:3)+relPositionCart;
%Transform pdf coordinates into cartesian coordiantes
xcart = zeros(3,length(x));
ycart = zeros(3,length(y));
for i = 1:length(x)
   xvec = [x(i);0;0];
   yvec = [0;y(i);0];
   xcart(:,i) = Q*xvec;
   ycart(:,i) = Q*yvec;
   xcart(:,i) = xcart(:,i)+satelliteState(1:3)'; %do more to fix dimensions
   ycart(:,i) = ycart(:,i)+satelliteState(1,3)';
end
burnTime = 30;
miss = true;
while miss && burnTime>0
    [maneuverPos,tAfter,stateAfter] = find_maneuver_position(satelliteState,burnTime,...
        direction,timeToCol);
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