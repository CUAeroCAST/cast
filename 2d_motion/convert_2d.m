function relScaledHill = convert_2d(refOrbit, conjOrbit, guidanceParams)
% Convert 3D orbit to 2D testbed plane
% This function converts the state vectors of two orbits in earth cartesian
% frame to a relitive hill frame on the 2D testbed
% Inputs:
% tRef: Time vector for the orbits
% refOrbit: Cartesian state vectors of the reference orbit
% conjOrbit: Cartesian state vectors of conjunction orbit
% Outputs:
% relScalledHill: relitive state vector in 2D testbead hill frame
% tRef: Time vector
% Author: Jason Balke | Project: CAST | Date: 10/20/20
%----------------------------------------------------------------------------------------%
% Get normal vector to collision plane and location of collision
scaling = guidanceParams.scaling;
planeNormalVec = cross(refOrbit(1,4:6),conjOrbit(2,4:6));
colPoint = [refOrbit(1,1:3) 0 0 0]; 
len = length(refOrbit);
ref2colPoint = zeros(len, 6);
conj2colPoint = zeros(len, 6);
ref2dPos = zeros(len, 3);
conj2dPos = zeros(len, 3);
ref2dVel = zeros(len, 3);
conj2dVel = zeros(len, 3);
relScaledHill = zeros(len, 4);

% Projecting onto 2D plane
for j = 1:length(refOrbit)
    % Get state vectors relitive to collision point
    ref2colPoint(j,:) = refOrbit(j,1:6)-colPoint;
    conj2colPoint(j,:) = conjOrbit(j,1:6)-colPoint;
    % Project the vectors onto the 2D plane
    ref2dPos(j,:) = ref2colPoint(j,1:3)-...
        (dot(ref2colPoint(j,1:3),planeNormalVec)/norm(planeNormalVec)^2)*planeNormalVec;
    conj2dPos(j,:) = conj2colPoint(j,1:3)-...
        (dot(conj2colPoint(j,1:3),planeNormalVec)/norm(planeNormalVec)^2)*planeNormalVec;
    ref2dVel(j,:) = ref2colPoint(j,4:6)-...
        (dot(ref2colPoint(j,4:6),planeNormalVec)/norm(planeNormalVec)^2)*planeNormalVec;
    conj2dVel(j,:) = conj2colPoint(j,4:6)-...
        (dot(conj2colPoint(j,4:6),planeNormalVec)/norm(planeNormalVec)^2)*planeNormalVec;
end
% Combine position and velocity into the full state vector
ref2d = [ref2dPos(:,1:2) ref2dVel(:,1:2)];
conj2d = [conj2dPos(:,1:2) conj2dVel(:,1:2)];
% Scale to testbed size
refScaled = ref2d/scaling;
conjScaled = conj2d/scaling;
% Get relitive position
relScaledCart = conjScaled-refScaled;
% Convert to the 2D Hill frame
for k = 1:len
    % Get Hill frame unit vectors
    scaledUnitAlong = refScaled(k,3:4)/norm(refScaled(k,3:4));
    along3d = [scaledUnitAlong 0];
    rad3d = [0 0 1];
    crossProduct = cross(along3d,rad3d);
    scaledUnitCross = crossProduct(1:2);
    % Transistion matrix
    Q = [scaledUnitAlong;scaledUnitCross];
    % Convert state vectors to Hill frame
    relScaledHill(k,1:2) = Q*relScaledCart(k,1:2)';
    relScaledHill(k,3:4) = Q*relScaledCart(k,3:4)';
end
end
