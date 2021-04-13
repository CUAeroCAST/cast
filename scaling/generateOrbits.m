function [t,chiefOrbit,collisionOrbit] = generateOrbits(relativeAngle,guidanceParams)
satelliteState = guidanceParams.chiefState';
v = satelliteState(4);
vCol = [v*cosd(relativeAngle) v*sind(relativeAngle) 0];
collisionState = [satelliteState(1:3) vCol];
satelliteState(4) = -satelliteState(4);
tspan = 30:-1:0;
mu = 398600;
[t,chiefOrbit] = ode45(@(t,chiefOrbit) orbit_prop(t,chiefOrbit,mu),tspan,satelliteState);
[t2,collisionOrbit] = ode45(@(t2,collisionOrbit)orbit_prop(t2,collisionOrbit,mu),tspan,collisionState);
end