function [alignedOrbit] = align_orbit(relativeOrbit)
%ALIGN_ORBIT This function takes in a cartesian relative orbit and aligns
%it with the x-axis for feeding into the sensor model
p = polyfit(relativeOrbit(:,1),relativeOrbit(:,2),1);
thetaZ = atand(p(1));
dcm = [cosd(thetaZ), -sind(thetaZ), 0;
       sind(thetaZ), cosd(thetaZ) , 0;
       0           , 0            , 1];
alignedOrbit = relativeOrbit * dcm;
end

