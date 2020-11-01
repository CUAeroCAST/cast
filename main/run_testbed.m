%% HOUSEKEEPING
clear; clc; close all
importCast;
datapath = open_logging();

%% CONFIG
gmatParams = struct;
estimatorParams = struct;
 estimatorParams.stepSize = 1;
 estimatorParams.initState = [0,0,0,0,0,0];
guidanceParams = struct;
plotStruct = struct;
recv = struct;

%Sensor parameters
sensorParams.samplingRate = 4e3;
sensorParams.maxRange = 4e3;
sensorParams.beamDivergence = 0.9; %deg
sensorParams.rangeAccuracy = 0.025; %m
sensorParams.beamLimits = [-0.45,0.45];
sensorParams.sensorType = 'Lidar';
sensorParams.scanRate = 10; %Hz

%Target parameters
targetParams.Mesh = extendedObjectMesh('sphere');
targetParams.Dimensions.Length = 40e-3; 
targetParams.Dimensions.Width = 40e-3;
targetParams.Dimensions.Height = 40e-3;
targetParams.Dimensions.OriginOffset = [0,0,0];

%Live Plot
plotStruct.filename = [datapath, filesep, '2D_collision_avoid'];
plotStruct.vobj = VideoWriter(plotStruct.filename, 'MPEG-4');
plotStruct.vobj.Quality = 100;
open(plotStruct.vobj);
plotStruct.videoFig = figure;
plotStruct.axis = [-1,20,-1,20];
plotStruct.collisionFlag = 0;   %flag for if covariance has intersected with gantry pos

%Save parameter structs
log_struct(gmatParams, [datapath, filesep, 'gmatParams'])
log_struct(estimatorParams, [datapath, filesep, 'estimatorParams'])
log_struct(guidanceParams, [datapath, filesep, 'guidanceParams'])
log_struct(plotStruct, [datapath, filesep, 'plotStruct'])
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
 
 % Visualization
 
 plotStruct = update_live_plot(plotStruct,estimate,recv,i);
 
 pause(real_time_delay)

end
%% CLEANUP
close_logging(plotStruct);
