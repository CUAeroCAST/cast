function [t,chiefOrbit,collisionOrbit] = generateOrbits(simParams, guidanceParams)
satelliteState = guidanceParams.chiefState';
relativeAngle = simParams.angle;
tspan = simParams.time : -1/simParams.rate : 0;
mu = simParams.muEarth;
v = satelliteState(4);
vCol = [v*cosd(relativeAngle) v*sind(relativeAngle) 0];
collisionState = [satelliteState(1:3) vCol];
satelliteState(4) = -satelliteState(4);
[t,chiefOrbit] = ode45(@(t, chiefOrbit) orbit_prop(t, chiefOrbit, mu), tspan, satelliteState);
[~,collisionOrbit] = ode45(@(t2, collisionOrbit)orbit_prop(t2, collisionOrbit, mu), tspan, collisionState);
t = flipud(t);
chiefOrbit = 1000*flipud(chiefOrbit);
collisionOrbit = 1000*flipud(collisionOrbit);
end