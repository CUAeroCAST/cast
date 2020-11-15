clear;close all;clc;importCast;
% Make Reference Orbit
altitude = 1200;
mu = 3.98600e5;
rad = 6378;
ref_state = make_reference(altitude, mu, rad);
[~, ref_semi_major, ~, ~, ~, ~] = elements_from_state(ref_state, mu);
T = orbit_period(mu, ref_semi_major);
n = 2 * pi / T;
int_period = [100, 0];
  
[ref_t, ref_orbit] = ...
 ode45(@(t, y) orbit_prop(t, y, mu), int_period, ref_state);

r0 = ref_orbit(end,1:3);
v0 = ref_orbit(end,4:6);
a0 = -(mu/norm(r0)^3)*r0;
j0 = -(mu/norm(r0)^3)*v0;
s0 = -(mu/norm(r0)^3)*a0;

r = r0+v0.*ref_t+(1/2).*a0.*ref_t.^2+(1/6).*j0.*ref_t.^3+(1/24).*s0.*ref_t.^4;
v = v0+a0.*ref_t+(1/2).*j0.*ref_t.^3+(1/6).*s0.*ref_t.^3;
jState = [r v];
magError = abs(jState(:,1:3)-ref_orbit(:,1:3));
for i = 1:3
    plot(magError(:,i))
    hold on
end
burnDirection = [0 1 0];
tspan = [0 30];
[tBurn, burnOrbit] = ode45(@(t, y) orbit_prop_maneuver(t, y, mu, burnDirection(1:2), 30),...
    tspan, [ref_orbit(end,:)';850]);
r0burn = ref_orbit(end,1:3);
v0burn = ref_orbit(end,4:6);
a0burn = -(mu/norm(r0burn)^3)*r0burn+(275/850)*burnDirection;
j0burn = -(mu/norm(r0burn)^3)*v0burn-(5.29496953160763/850)*burnDirection;
s0burn = -(mu/norm(r0burn)^3)*a0burn;

rburn = r0burn+v0burn.*tBurn+(1/2).*a0burn.*tBurn.^2+(1/6).*j0burn.*tBurn.^3+(1/24).*s0burn.*tBurn.^4;
vburn = v0burn+a0burn.*tBurn+(1/2).*j0burn.*tBurn.^3+(1/6).*s0burn.*tBurn.^3;

burnError = abs(rburn-burnOrbit(:,1:3));

for i = 1:3
    plot(burnError(:,i))
    hold on
end
