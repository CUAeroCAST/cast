clear
clc
close all
importCast;

global datapath;
%datapath = open_logging(true);

estimatorParams = make_estimator_params();
estimatorParams.currentTime = 0;
guidanceParams = make_guidance_params();
simParams = make_sim_params();

[t, chiefOrbit, collisionOrbit] = generateOrbits(simParams, guidanceParams);
[sim, sensor, estimatorParams.filter.State] = make_simulation(t, chiefOrbit, collisionOrbit, simParams);
[t, m] = make_sensor_readings(sim, sensor);

collisionEstimate(1) = struct("collisionTime", nan, "predState", nan, "Ppred", nan);

steps = length(t);
estimateStorage(steps) = struct("corrState", nan, "Pcorr", nan, "predState", nan, "Ppred", nan);
collisionStorage(steps) = struct("collisionTime", nan, "predState", nan, "Ppred", nan);
measurementStorage(steps) = struct("time", nan, "distance", nan, "angle", nan);

moving = false;
for i = 1 : steps
 measurement.time = t(i);
 measurement.distance = m(i, 1);
 measurement.angle = 0;
 
 [estimate, estimatorParams] = state_estimator([measurement.distance; measurement.angle],...
                                        t(i), estimatorParams);
 collisionEstimate = collision_prediction(estimate, estimatorParams, collisionEstimate);
 if ~moving
  maneuver = make_maneuver(collisionEstimate, guidanceParams);
  if maneuver(3)
   moving = true;
  end
 end
end