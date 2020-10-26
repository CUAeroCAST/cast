%% 2d motion conversion
% This is a script that makes the 2d motion, it'll become a function at
% some point
clear;close all;clc;
importCast
%% Generate Orbits - This should be replaced with GMAT generated orbits
% Make Reference Orbit
altitude = 1200;
mu = 3.98600e5;
rad = 6378;
refState = make_reference(altitude, mu, rad);
[~, refSemiMajor, ~, ~, ~, ~] = elements_from_state(refState, mu);
T = orbit_period(mu, refSemiMajor);
n = 2 * pi / T;
intPeriod = [100, 0];
  
[tRef, refOrbit] = ...
 ode45(@(t, y) orbit_prop(t, y, mu), intPeriod, refState);
% Make conjunction
conjState = make_conjunction(altitude, mu, rad);
[~, conjOrbit] = ...
 ode45(@(t, y) orbit_prop(t, y, mu), tRef, conjState);
relCart = (conjOrbit - refOrbit)';
% Get coordinate transform for cartesian to hill
refOrbit = refOrbit';
unit_rad = refOrbit(1:3, :) ./ vecnorm(refOrbit(1:3, :));
unit_along = refOrbit(4:6, :) ./ vecnorm(refOrbit(4:6, :));
unit_cross = cross(unit_rad, unit_along);

hillVecs = zeros(size(relCart));
for i = 1:length(tRef)
 Q = [unit_rad(:, i)';
   unit_along(:, i)';
   unit_cross(:, i)'];
 Q = blkdiag(Q, Q);
 hillVecs(:, i) = Q * relCart(:, i);
end
unitRelVels = hillVecs(4:6, :) ./ vecnorm(hillVecs(4:6, :));
%% Converting to 2d plane
refOrbit = refOrbit';
[tRef, relScaledHill] = convert_2d(tRef,refOrbit,conjOrbit);
[t100,predictedPath100] = predict_path(tRef,relScaledHill,100,1);