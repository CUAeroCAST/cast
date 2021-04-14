function [ydot vdot] = orbit_prop_maneuver(t, y, mu, direction, burnTime)
% propogates the undisturbed two body equation
 r = y(1:3);
 v = y(4:6);
 m = y(7);
 mdot = 0;
 Isp = 225;
 g0 = 9.81;
 Tslope = -5.29496953160763;
%  direction = [direction 0];
 thrustUnitVec = direction/norm(direction);
 if isnan(thrustUnitVec)
     thrustUnitVec = [0;0;0];
 end
 rdot = v;
 if t>burnTime
     vdot = -mu*r/norm(r)^3;
 else
     T = Tslope*t+275;   %slope curve fitted from thruster model
     mdot = -T/(g0*Isp);
     vdot = (-mu*r/norm(r)^3)+(T/m)*thrustUnitVec;
 end
 

 ydot = [rdot; vdot; mdot];
end