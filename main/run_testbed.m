%% HOUSEKEEPING
clear; clc; close all
importCast;
open_logging();

%% CONFIG
gmatParams = struct;
sensorParams = struct;
estimatorParams = struct;
 estimatorParams.stepSize = 1;
guidanceParams = struct;

%% GMAT

[chiefOrbit, deputyOrbit, timeVec] = make_gmat_orbits(gmatParams);

relativeOrbit = chiefOrbit - deputyOrbit;

%% SENSOR MODEL

sensorScenario = init_sensor_model(relativeOrbit, sensorParams);
sensorReadings = sensor_model(sensorScenario);

%% STATE ESTIMATION

[offset, estimatorParams] = init_estimator(sensorReadings, estimatorParams);

%% MAIN LOOP
for i = offset : estimatorParams.stepSize : length(timeVec)
 % STATE ESTIMATION
 sensorReading = sensorReadings(i);
 time = timeVec(i);
 real_time_delay = 0.01;
 
 
 [estimate, estimatorParams] = state_estimator(sensorReading, time,...
                                                      estimatorParams);

 % GUIDANCE

 [maneuver, delay] = make_maneuver(estimate, guidanceParams);
 
 recv = run_io(maneuver, delay);
 
 update_live_plot(recv);
 
 pause(real_time_delay)

end
%% CLEANUP

close_logging();