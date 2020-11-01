%% HOUSEKEEPING
clear; clc; close all
importCast;
datapath = open_logging();

%% CONFIG
scalingFactor = 222; %Distance scaling factor for testbed

gmatParams = struct;
estimatorParams = struct;
 estimatorParams.stepSize = 1;
guidanceParams = struct;
plotStruct = struct;
recv = struct;

%Sensor parameters
makeSensorPlot = false;

sensorParams.samplingRate = 4e3;
sensorParams.maxRange = 4e3;
sensorParams.beamDivergence = 0.9; %deg
sensorParams.rangeAccuracy = 0.025; %m
sensorParams.beamLimits = [-1.35,1.35];
sensorParams.sensorType = 'Lidar';
sensorParams.scanRate = 10; %Hz

%Target parameters
targetParams.Mesh = extendedObjectMesh('sphere');
targetParams.Dimensions.Length = 40e-3; 
targetParams.Dimensions.Width = 40e-3;
targetParams.Dimensions.Height = 40e-3;
targetParams.Dimensions.OriginOffset = [0,0,0];

%Avoidance Parameters
canAvoid = false;

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

%Temp creation of relative orbit for init_sensor_model
load('relativeOrbitExample.mat');
relativeOrbit = align_orbit(relativeOrbit);
relativeOrbit = relativeOrbit.*(1000/scalingFactor);
%Trim values outside sensor range
[~,I] = min(abs(relativeOrbit(:,1) - sensorParams.maxRange));
relativeOrbit(1:I,:) = [];

n = length(relativeOrbit);
%This needs to be corrected
timeVec = linspace(0,n/10,n)';
%% SENSOR MODEL

sensorScenario = init_sensor_model(relativeOrbit, timeVec, sensorParams,...
                                   targetParams);
sensorReadings = sensor_model(sensorScenario, makeSensorPlot);

%% STATE ESTIMATION
%Determine how many sensor readings to use for batch LLS estimate
initialReadings = sensorReadings(1:10,:);
[offset, estimatorParams] = init_estimator(initialReadings, estimatorParams);
estimatorParams.currentTime = timeVec(offset);


%% MAIN LOOP
for i = offset : estimatorParams.stepSize : length(timeVec)
 % STATE ESTIMATION
 sensorReading = sensorReadings(i,:);
 time = timeVec(i);
 real_time_delay = 0;
 
 [estimate, estimatorParams] = state_estimator(sensorReading, time,...
                                                      estimatorParams);
 
 %This needs to be set when the covariance elipse is within bounds, only
 if(estimate.corrState(2)<0)
  collisionTime = -estimate.corrState(1)/estimate.corrState(2) + ...
                  estimatorParams.currentTime;
  %need to calculate it when we can avoid
  collisionEstimate = desync_predict(collisionTime, estimatorParams); 
 end
 
 % GUIDANCE

 [maneuver, delay] = make_maneuver(estimate, guidanceParams);
 
 recv = run_io(maneuver, delay);
 
 % Visualization
 
 plotStruct = update_live_plot(plotStruct,estimate,recv,i);
 
 pause(real_time_delay)

end
%% CLEANUP
close_logging(plotStruct);
