%% HOUSEKEEPING
clear; clc; close all
importCast;
datapath = open_logging();

%% CONFIG
makeSensorPlot = false;
loadFile = true;

gmatParams = struct;
guidanceParams = struct;
plotStruct = struct;
recv = struct;

%Simulation parameters
simulationParams.stepSize = 1;
simulationParams.sampleRate = 1e3;
simulationParams.scalingFactor = 222; %Distance scaling factor for testbed

%Estimator parameters
estimatorParams.llsSeeding = true;
estimatorParams.batchSamples = 10;

%Sensor parameters
sensorParams.samplingRate = 1e3;
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
%% GMAT

[chiefOrbit, deputyOrbit, timeVec] = make_gmat_orbits(gmatParams);

%Temp creation of relative orbit for init_sensor_model
load('relativeOrbitExample.mat');
relativeOrbit = align_orbit(relativeOrbit);
relativeOrbit = relativeOrbit./simulationParams.scalingFactor;
%Trim values outside sensor range
[~,I] = min(abs(relativeOrbit(:,1) - sensorParams.maxRange));
relativeOrbit(1:I,:) = [];
n = length(relativeOrbit);
timeVec = linspace(0,n/simulationParams.sampleRate,n)';
%% SENSOR MODEL
if(loadFile)
    load('sensorReadings.mat');
else
    sensorScenario = init_sensor_model(relativeOrbit, timeVec, sensorParams,...
                                   targetParams);
    sensorReadings = sensor_model(sensorScenario, makeSensorPlot);
end

%% STATE ESTIMATION
%Determine how many sensor readings to use for batch LLS estimate
[offset, estimatorParams] = init_estimator(sensorReadings, estimatorParams);
estimatorParams.currentTime = timeVec(offset);


%% MAIN LOOP
for i = offset : simulationParams.stepSize : length(timeVec)
 % STATE ESTIMATION
 sensorReading = sensorReadings(i,:);
 time = timeVec(i);
 real_time_delay = 0;
 
 [estimate, estimatorParams] = state_estimator(sensorReading, time,...
                                                      estimatorParams);
 
 %Forward prediction when collision time can be predicted
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
