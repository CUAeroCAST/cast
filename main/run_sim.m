clear
clc
close all
importCast;

global datapath;
datapath = open_logging(true);

estimatorParams = make_estimator_params();
estimatorParams.currentTime = 0;
guidanceParams = make_guidance_params();
simParams = make_sim_params();

log_struct(estimatorParams, [datapath, filesep, 'estimatorParams'])
log_struct(guidanceParams, [datapath, filesep, 'guidanceParams'])
log_struct(simParams, [datapath, filesep, 'simParams'])

[t, chiefOrbit, collisionOrbit] = generateOrbits(simParams, guidanceParams);
[sim, sensor, estimatorParams.filter.State] = make_simulation(t, chiefOrbit, collisionOrbit, simParams);
[t, m] = make_sensor_readings(sim, sensor, simParams);

collisionEstimate(1) = struct("collisionTime", nan, "predState", nan, "Ppred", nan);

steps = length(t);
estimateStorage(steps) = struct("corrState", nan, "Pcorr", nan, "predState", nan, "Ppred", nan);
collisionStorage(steps) = struct("collisionTime", nan, "predState", nan, "Ppred", nan);
measurementStorage(steps) = struct("time", nan, "distance", nan, "angle", nan);

moving = false;
for i = 1 : steps
 measurement.time = t(i);
 measurement.distance = m(i, 1);
 measurement.angle = m(i, 2);
 
 [estimate, estimatorParams] = state_estimator([measurement.distance; measurement.angle],...
                                        t(i), estimatorParams);
 collisionEstimate = collision_prediction(estimate, estimatorParams, collisionEstimate);
 if ~moving
  timeToCol = t(end) - t(i);%collisionEstimate.collisionTime - t(i);
  [maneuver, tManeuver, stateManeuver] = make_maneuver_sim(collisionEstimate, [chiefOrbit(i,:), simParams.mass], timeToCol, guidanceParams);
  if maneuver(3)
   moving = true;
  end
 end
 estimateStorage(i) = estimate;
 collisionStorage(i) = collisionEstimate;
 measurementStorage(i) = measurement;
end

log_struct(estimateStorage, [datapath, filesep, 'estimateStorage'])
log_struct(collisionStorage, [datapath, filesep, 'collisionStorage'])
log_struct(measurementStorage, [datapath, filesep, 'measurementStorage'])
log_struct(maneuver, [datapath, filesep, 'maneuver'])
log_struct(stateManeuver, [datapath, filesep, 'maneuveredOrbit'])
log_struct(chiefOrbit, [datapath, filesep, 'chiefOrbit'])
log_struct(collisionOrbit, [datapath, filesep, 'collisionOrbit'])