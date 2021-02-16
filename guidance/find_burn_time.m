function [burnTime,tAfter,stateAfter] = find_burn_time(est,satelliteState,direction,timeToCol,burnTimesteps)
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
est.Pcorr = est.Ppred;
est.corrState = est.predState;
scaling = 222;
est.corrState = est.corrState*scaling;
est.Pcorr = est.Pcorr*scaling^2;
% Get the position of the debris
relPositionHill = [est.corrState(1) est.corrState(2) 0]';
colState = [217.547021908829,0,7574.87672106969,7.24957465758464,0,-0.208204291222004];
% Convert to orbital scale
%5 Oct 2020 00:26:51.325 434.915387 0.000000 12.490366 -14.481090 -0.000000 -0.832453 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% maxVel = .5;
% maxDist = 1.5;
% maxOrbitVel = 14.50512e3;
% maxOrbitDist = 434.915387;
% scalingFactorVel = maxOrbitVel/maxVel;
% scalingFactorDist = maxOrbitDist/maxDist;
% estimate.corrState(1) = estimate.corrState(1)*scalingFactorDist;
% estimate.corrState(3) = estimate.corrState(3)*scalingFactorDist;
% estimate.corrState(2) = estimate.corrState(2)*scalingFactorVel;
% estimate.corrState(4) = estimate.corrState(4)*scalingFactorVel;
% estimate.Pcorr(1,1) = estimate.Pcorr(1,1)*scalingFactorDist^2;
% estimate.Pcorr(3,3) = estimate.Pcorr(3,3)*scalingFactorDist^2;
% estimate.Pcorr(2,2) = estimate.Pcorr(2,2)*scalingFactorVel^2;
% estimate.Pcorr(4,4) = estimate.Pcorr(4,4)*scalingFactorVel^2;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Calculating the pdf
xmax = est.corrState(1)+2*sqrt(est.Pcorr(1,1));
xmin =  est.corrState(1)-2*sqrt(est.Pcorr(1,1));
ymax = est.corrState(3)+2*sqrt(est.Pcorr(3,3));
ymin =  est.corrState(3)-2*sqrt(est.Pcorr(3,3));
% Transform the debris position
unitRad = satelliteState(1:3)/norm(satelliteState(1:3));
unitAlong = satelliteState(4:6)/norm(satelliteState(4:6));
unitCross = cross(unitRad,unitAlong);
Q = [unitRad';unitAlong';unitCross'];  % make sure this Q goes the right way
relPositionCart = Q'*relPositionHill;
cartState = satelliteState(1:3)+relPositionCart;
%Transform pdf coordinates into cartesian coordiantes
xmaxCart = Q'*[0;xmax;0]+colState(1);
xminCart = Q'*[0;xmin;0]+colState(1);
ymaxCart = Q'*[0;0;ymax]+colState(2);
yminCart = Q'*[0;0;ymin]+colState(2);
burnTime = 30;
miss = true;
while miss && burnTime>0
%     colTimeFull = sqrt(relPositionCart(1)^2+relPositionCart(2)^2)/...
%         sqrt(estimate.corrState(2)^2+estimate.corrState(4)^2);
    [maneuverPos,tAfter,stateAfter] = find_maneuver_position(satelliteState,burnTime,...
        direction,30,burnTimesteps);
%     maneuuverPosRel = maneuverPos-satelliteState(1:3);
%     maneuverPosHill = Q'*maneuuverPosRel';
    if (maneuverPos(1)>xminCart(1) && maneuverPos(1)<xmaxCart(1)) && (maneuverPos(2)>yminCart(2) && maneuverPos(2)<ymaxCart(2))
        miss = false;
        burnTime = burnTime+1;
        break
    else
        miss = true;
    end
    burnTime = burnTime-1;
end
end
