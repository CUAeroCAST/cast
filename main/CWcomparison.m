clear;close all;clc;
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
% Make conjunction
conj_state = make_conjunction(altitude, mu, rad);
[~, conj_orbit] = ...
 ode45(@(t, y) orbit_prop(t, y, mu), ref_t, conj_state);
relCart = (conj_orbit - ref_orbit)';
% Get coordinate transform for cartesian to hill
ref_orbit = ref_orbit';
unit_rad = ref_orbit(1:3, :) ./ vecnorm(ref_orbit(1:3, :));
unit_along = ref_orbit(4:6, :) ./ vecnorm(ref_orbit(4:6, :));
unit_cross = cross(unit_rad, unit_along);

hillVecs = zeros(size(relCart));
for i = 1:length(ref_t)
 Q = [unit_rad(:, i)';
   unit_along(:, i)';
   unit_cross(:, i)'];
 Q = blkdiag(Q, Q);
 hillVecs(:, i) = Q * relCart(:, i);
end
unitRelVels = hillVecs(4:6, :) ./ vecnorm(hillVecs(4:6, :));

curvHillVecs = curvilinear_hill_vecs(hillVecs, rad + altitude);
CWsolution = zeros(size(hillVecs));
curviCWsolution = CWsolution;
for j = 1:length(ref_t)
 t = ref_t(1) - ref_t(j);
 hill_matrix = make_hill_matrix(n, t);
 CWsolution(:, j) = hill_matrix * hillVecs(:, j);
 curviCWsolution(:, j) = hill_matrix * curvHillVecs(:, j);
end



CWError = vecnorm(cross(CWsolution(1:3, :), unitRelVels)) * 1000;
curvCWError = sphError(curviCWsolution(1:3, :), rad + altitude, unitRelVels) * 1000;
rel_distance = vecnorm(relCart(1:3, :));
valid = rel_distance < 150;

figure
plot3(ref_orbit(1,:), ref_orbit(2,:), ref_orbit(3,:),...
conj_orbit(:,1), conj_orbit(:,2), conj_orbit(:,3))
hold on
[X, Y, Z] = sphere();
surf(X*rad, Y*rad, Z*rad)

figure
hold on
grid minor
plot(rel_distance(valid), CWError(valid), rel_distance(valid), curvCWError(valid),...
  'LineWidth', 2)
ax = gca;
ax.GridAlpha = 1;
ax.LineWidth = 1;
title('Collision Plane Error Introduced by CW Equations')
xlabel('Initial Relative Distance (km)')
ylabel('Collision Plane Error (m)')
legend('Cartesian CW', 'Spherical CW')



