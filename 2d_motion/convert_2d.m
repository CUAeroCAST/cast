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
planeNormalVec = cross(refOrbit(2,4:6), conjOrbit(2,4:6));
colPoint = [refOrbit(end, 1:3), 0, 0, 0]; 
len = length(refOrbit);

relScaledHill = zeros(len, 4);

% Projecting onto 2D plane
ref2colPoint = refOrbit(:, 1:6) - colPoint;
conj2colPoint = conjOrbit(:, 1:6) - colPoint;
ref2dPos = ref2colPoint(:, 1:3) - dot(ref2colPoint(:, 1:3), repmat(planeNormalVec, len, 1), 2) / norm(planeNormalVec)^2 * planeNormalVec;
conj2dPos = conj2colPoint(:, 1:3) - dot(ref2colPoint(:, 1:3), repmat(planeNormalVec, len, 1), 2) / norm(planeNormalVec)^2 * planeNormalVec;
ref2dVel =  ref2colPoint(:, 4:6) - dot(ref2colPoint(:, 4:6), repmat(planeNormalVec, len, 1), 2) / norm(planeNormalVec)^2 * planeNormalVec;
conj2dVel = conj2colPoint(:, 4:6) - dot(ref2colPoint(:, 4:6), repmat(planeNormalVec, len, 1), 2) / norm(planeNormalVec)^2 * planeNormalVec;


% Combine position and velocity into the full state vector
ref2d = [ref2dPos(:, 1:2), ref2dVel(:, 1:2)];
conj2d = [conj2dPos(:, 1:2), conj2dVel(:, 1:2)];

% Scale to testbed size
refScaled = ref2d/scaling;
conjScaled = conj2d/scaling;

% Get relative position
relScaledCart = conjScaled-refScaled;

scaledUnitAlong = refScaled(1, 3:4) ./ vecnorm(refScaled(1, 3:4), 2, 2);
along3d = [scaledUnitAlong, 0];
rad3d = [0, 0, 1];
crossProduct = cross(along3d, rad3d);
scaledUnitCross = crossProduct(1:2);
Q = [scaledUnitAlong; scaledUnitCross];
relScaledHill(:, 1:2) = (Q * relScaledCart(:, 1:2)')';
relScaledHill(:, 3:4) = (Q * relScaledCart(:, 3:4)')';

end
