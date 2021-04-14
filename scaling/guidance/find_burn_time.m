function [burnTime,tAfter,stateAfter] = find_burn_time(estimate, direction)
estimate.Pcorr = estimate.Ppred;
estimate.corrState = estimate.predState;
scaling = 222;
estimate.corrState = estimate.corrState*scaling;
estimate.Pcorr = estimate.Pcorr*scaling^2;
% Get the position of the debris
relPositionHill = [estimate.corrState(1) estimate.corrState(2) 0]';
colState = [0;0;7578;-7.25256299066873;0;0];
% Calculating the pdf
xmax = estimate.corrState(1)+2*sqrt(estimate.Pcorr(1,1));
xmin =  estimate.corrState(1)-2*sqrt(estimate.Pcorr(1,1));
ymax = estimate.corrState(3)+2*sqrt(estimate.Pcorr(3,3));
ymin =  estimate.corrState(3)-2*sqrt(estimate.Pcorr(3,3));
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
endPos = colState;
for i = 1:30
    maneuverPos = endPos{i};
    if (maneuverPos(1)>xminCart(1) && maneuverPos(1)<xmaxCart(1)) && (maneuverPos(2)>yminCart(2) && maneuverPos(2)<ymaxCart(2))
        continue
    else
        [tAfter,stateAfter] = ode45(@(tAfter,stateAfter)...
            orbit_prop(tAfter,stateAfter,mu,direction,i),0:timeToCol,satelliteState);
        maneuverPos = stateAfter(end,:);
        break
    end
end
burnTime = i;
end
