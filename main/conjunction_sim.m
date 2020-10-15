% Monte Carlo Simulation of spacecraft conjunction statistics
% For Senior Projects - ULA Collision Avoidance System
% Author: Trace Valade
% Date: Sept 6, 2020
%% HOUSEKEEPING
clear
close all
clc

%% DRIVER
mu = 398600; % km^3/s^2
rad = 6378; % km
num_samples = 10000;

planar_conjunction_data = lo_monte_carlo_conjunction(mu, rad, num_samples);
save('planar_conjunction_data_with_geo', 'planar_conjunction_data')

%% MONTE CARLO
function output = lo_monte_carlo_conjunction(mu, rad, num_samples)
% runs a monte carlo simulation for spacecraft conjunctions in low orbit
% altitudes run from 400 to 1200
% generates a reference circular polar orbit
% then generates num_samples random intersecting orbits
% and checks for <5deg cosine distance from final approach vector
% then calculates the time to conjunction, maximum distance, and minimum view angle for
% <5deg cosine distance and a 1m diameter target.
% Then calculates time to conjunction and maximum distance for a 1deg view angle.
% Outputs are averaged over num_samples at each altitude and returned as a struct.
 
 altitudes = [400 : 100 : 1200, 35786];
 empty_cell = cell(1, length(altitudes));
 output = struct('altitude', empty_cell, 'mean_planar_time', 0,...
  'mean_planar_distance', 0, 'mean_planar_view_angle', 0,...
  'mean_one_arcm_time', 0, 'mean_one_arcm_distance', 0,...
  'min_planar_time', 1e5,'min_planar_distance', 1e5,...
  'max_planar_time', 0, 'max_planar_distance', 0,...
  'mean_conj_angle', 0, 'min_conj_angle', 200,...
  'max_conj_angle', 0);
 
 i = 1;
 
 for altitude = altitudes

  ref_state = make_reference(altitude, mu, rad);
  [~, ref_semi_major, ~, ~, ~, ~] = elements_from_state(ref_state, mu);
  T = orbit_period(mu, ref_semi_major);
  int_period = [T/4, 0];
  
  [ref_t, ref_orbit] = ...
   ode45(@(t, y) orbit_prop(t, y, mu), int_period, ref_state);
  j = 0;
  while j < num_samples
   conj_state = make_conjunction(altitude, mu, rad);

   [~, conj_orbit] = ...
    ode45(@(t, y) orbit_prop(t, y, mu), ref_t, conj_state);
%    
%    figure
%    plot3(ref_orbit(:,1), ref_orbit(:,2), ref_orbit(:,3),...
%     conj_orbit(:,1), conj_orbit(:,2), conj_orbit(:,3))
%    hold on
%    [X, Y, Z] = sphere();
%    surf(X*rad, Y*rad, Z*rad)
   

   [planar_time,...
   planar_distance,...
   planar_view_angle,...
   one_arcm_time,...
   one_arcm_distance,...
   conj_angle] = calc_statistics(ref_orbit, ref_t, conj_orbit, rad, T);
   
   if (planar_time > 0) && (conj_angle > 135)
    output(i).mean_planar_time = ...
     output(i).mean_planar_time + planar_time;
    
    output(i).mean_planar_distance = ...
     output(i).mean_planar_distance + planar_distance;
    
    output(i).mean_planar_view_angle = ...
     output(i).mean_planar_view_angle + planar_view_angle;
    
    output(i).mean_one_arcm_time = ...
     output(i).mean_one_arcm_time + one_arcm_time;
    
    output(i).mean_one_arcm_distance = ...
     output(i).mean_one_arcm_distance + one_arcm_distance;
    
    output(i).max_planar_time = ...
     max(planar_time, output(i).max_planar_time);
    output(i).max_planar_distance = ...
     max(planar_distance, output(i).max_planar_distance);
    
    output(i).min_planar_time = ...
     min(planar_time, output(i).min_planar_time);
    output(i).min_planar_distance = ...
     min(planar_distance, output(i).min_planar_distance);
    
    output(i).mean_conj_angle = ...
     output(i).mean_conj_angle + conj_angle;
    output(i).min_conj_angle = ...
     min(conj_angle, output(i).min_conj_angle);
    output(i).max_conj_angle = ...
     max(conj_angle, output(i).max_conj_angle);
    
    j = j + 1;
   end
  end
  
  output(i).altitude = altitude;
  
  output(i).mean_planar_time = ...
   output(i).mean_planar_time / num_samples;
  
  output(i).mean_planar_distance = ...
   output(i).mean_planar_distance / num_samples;
  
  output(i).mean_planar_view_angle = ...
   output(i).mean_planar_view_angle / num_samples;
  
  output(i).mean_one_arcm_time = ...
   output(i).mean_one_arcm_time / num_samples;
  
  output(i).mean_one_arcm_distance = ...
   output(i).mean_one_arcm_distance / num_samples;
  
  output(i).mean_conj_angle = ...
   output(i).mean_conj_angle / num_samples;
  
  i = i + 1;
 end
end