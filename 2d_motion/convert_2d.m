function colVels = convert_2d(refOrbit, burnOrbit, guidanceParams)
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
% Updated: Trace Valade | Project: CAST| Date: 4/1/21
%  Was previously transforming into an incorrect coordinate system.
%----------------------------------------------------------------------------------------%
% Get normal vector to collision plane and location of collision
scaling = guidanceParams.scaling;
colOrbit = guidanceParams.deputyState;
len = length(burnOrbit);

% get the coordinate frame defined by the collision velocity and position
% at first detection (30s from collision).
colYVec = colOrbit(1:3)' - refOrbit(1, 1:3);
colYVec = colYVec / norm(colYVec);
colZVec = cross(colOrbit(4:6), refOrbit(1, 4:6));
colZVec = colZVec - dot(colYVec, colZVec) * colYVec;
colZVec = colZVec / norm(colZVec);
colXVec = cross(colYVec, colZVec);



% Projecting onto 2D plane, we care about the relative velocity between the
% reference chief orbit and the burn orbit as this is the velocity that we
% can represent on the test bed starting at a stand still.
relVels = burnOrbit(:, 4:6) - refOrbit(:, 4:6);
colXVels = dot(relVels, repmat(colXVec, len, 1), 2);
colYVels = dot(relVels, repmat(colYVec, len, 1), 2);

% rescale the distances to fit on the test bed
colVels = [colXVels, colYVels] / scaling;

end
