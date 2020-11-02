function ydot = orbit_prop_maneuver(t, y, mu, direction, burnTime)
% propogates the undisturbed two body equation
 r = y(1:3);
 v = y(4:6);
 direction = [direction 0];
 thrustUnitVec = direction'/norm(direction);
 rdot = v;
 if t>burnTime
     vdot = -mu*r/norm(r)^3;
 else
     vdot = (-mu*r/norm(r)^3)+(275/850)*thrustUnitVec;
 end

 ydot = [rdot; vdot];
end