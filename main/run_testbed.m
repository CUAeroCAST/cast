%% HOUSEKEEPING
clear; clc; close all
importCast;
open_logging();

%% CONFIG
gmatParams = struct;
estimatorParams = struct;
 estimatorParams.stepSize = 1;
 estimatorParams.initState = [0,0,0,0,0,0];
guidanceParams = struct;

%Sensor parameters
sensorParams.samplingRate = 10;
sensorParams.maxRange = 4e3;
sensorParams.beamDivergence = 0.5; %deg
sensorParams.rangeAccuracy = 0.025; %m
sensorParams.beamLimits = [-0.25,0.25];
sensorParams.sensorType = 'Lidar';

%Target parameters
targetParams.Mesh = extendedObjectMesh('sphere');
targetParams.Dimensions.Length = 4.5e-3; 
targetParams.Dimensions.Width = 4.5e-3;
targetParams.Dimensions.Height = 4.5e-3;
targetParams.Dimensions.OriginOffset = [0,0,0];

%% GMAT

[chiefOrbit, deputyOrbit, timeVec] = make_gmat_orbits(gmatParams);

relativeOrbit = chiefOrbit - deputyOrbit;
%Temp creation of relative orbit for init_sensor_model
relativeOrbit = zeros(3,3);
timeVec = [0,1,2];

%% SENSOR MODEL

sensorScenario = init_sensor_model(relativeOrbit, timeVec, sensorParams,...
                                   targetParams);
sensorReadings = sensor_model(sensorScenario);

%% STATE ESTIMATION

[offset, estimatorParams] = init_estimator(sensorReadings, estimatorParams);
estimatorParams.currentTime = timeVec(offset);

%% MAIN LOOP
for i = offset : estimatorParams.stepSize : length(timeVec)
 % STATE ESTIMATION
 sensorReading = sensorReadings{i};
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