function burnTime = find_burn_time(Q, estimate, positionTable, directionIndex, guidanceParams)
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

scaling = guidanceParams.scaling;
estimate.corrState = estimate.corrState * scaling;
estimate.Pcorr = estimate.Pcorr * scaling^2;
% Get the position of the collision
colState = [217.547021908829, 0, 7574.87672106969, 7.24957465758464, 0, -0.208204291222004];
% Convert to orbital scale
%5 Oct 2020 00:26:51.325 434.915387 0.000000 12.490366 -14.481090 -0.000000 -0.832453 

% Calculating the pdf
xmax = estimate.corrState(1)+2*sqrt(estimate.Pcorr(1,1));
xmin =  estimate.corrState(1)-2*sqrt(estimate.Pcorr(1,1));
ymax = estimate.corrState(2)+2*sqrt(estimate.Pcorr(2,2));
ymin =  estimate.corrState(2)-2*sqrt(estimate.Pcorr(2,2));


%Transform pdf coordinates into cartesian coordiantes
xmaxCart = Q*[0;xmax;0]+colState(1);
xminCart = Q*[0;xmin;0]+colState(1);
ymaxCart = Q*[0;0;ymax]+colState(2);
yminCart = Q*[0;0;ymin]+colState(2);

burnTime = 30;
miss = true;
while miss && burnTime>0
    maneuverPos = positionTable{burnTime,directionIndex};
    if (maneuverPos(1)>xminCart(1) && maneuverPos(1)<xmaxCart(1)) && (maneuverPos(2)>yminCart(2) && maneuverPos(2)<ymaxCart(2))
        burnTime = min(burnTime+1, 30);
        break
    else
        miss = true;
    end
    burnTime = burnTime-1;
end
end
