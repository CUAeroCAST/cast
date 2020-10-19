%% 2d motion conversion
% This is a script that makes the 2d motion, it'll become a function at
% some point
clear;close all;clc;
%% Generate Orbits
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
%% Defining 2d plane

%% Converting to 2d plane

%% Scaling