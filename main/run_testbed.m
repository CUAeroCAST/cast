%% HOUSEKEEPING
clear; clc; close all
importCast;
datapath = open_logging();

%% CONFIG
makeSensorPlot = false;
loadFile = true;
isOrbitalScenario = false;
isTestbedScenario = true;

gmatParams = struct;
guidanceParams = struct;
plotStruct = struct;
recv = struct;

%Simulation parameters
simulationParams.stepSize = 1;
simulationParams.sampleRate = 4e3;
simulationParams.scalingFactor = 1; %Distance scaling factor for testbed
simulationParams.initPos = [2,0.5,0]; %m, starting point of object
simulationParams.finalPos = [0,0,0]; %m, final point of object
simulationParams.collisionTime = 2; %s, time it takes to get from initial 
%to final position

%Estimator parameters
estimatorParams.llsSeeding = true;
estimatorParams.batchSamples = 10;
estimatorParams.sensorCovariance = [0.025^2,0,0;0,0.2^2,0;0,0,0.2^2];

%Sensor parameters
sensorParams.samplingRate = 4e3;
sensorParams.maxRange = 15;
sensorParams.beamDivergence = 0.9; %deg
sensorParams.rangeAccuracy = 0.025; %m
sensorParams.beamLimits = [-1.35,1.35];
sensorParams.sensorType = 'Lidar';
sensorParams.scanRate = 10; %Hz

%Target parameters
targetParams.Mesh = extendedObjectMesh('sphere');
targetParams.Dimensions.Length = 50e-3; 
targetParams.Dimensions.Width = 50e-3;
targetParams.Dimensions.Height = 50e-3;
targetParams.Dimensions.OriginOffset = [0,0,0];

%Avoidance Parameters
canAvoid = false;

%Live Plot
plotStruct.filename = [datapath, filesep, '2D_collision_avoid'];
plotStruct.vobj = VideoWriter(plotStruct.filename, 'MPEG-4');
plotStruct.vobj.Quality = 100;
open(plotStruct.vobj);
plotStruct.videoFig = figure;
plotStruct.collisionFlag = 0;   %flag for if covariance has intersected with gantry pos
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
plotStruct.interval = 50; %how often the plot is updated... if equals 0, then it plots every iteration

%Save parameter structs
log_struct(gmatParams, [datapath, filesep, 'gmatParams'])
log_struct(estimatorParams, [datapath, filesep, 'estimatorParams'])
log_struct(guidanceParams, [datapath, filesep, 'guidanceParams'])
%% GMAT
if(isOrbitalScenario && ~loadFile)
 [chiefOrbit, deputyOrbit, timeVec] = make_gmat_orbits(gmatParams);
 relativePath = align_orbit(chiefOrbit-deputyOrbit);
 relativePath = relativePath./simulationParams.scalingFactor;
 [~,I] = min(abs(relativePath(:,1) - sensorParams.maxRange));
 relativePath(1:I,:) = [];
 n = length(relativePath);
 %timeVec = linspace(0,n/simulationParams.sampleRate,n)';
end

%% TESTBED COLLISION
if (isTestbedScenario && ~loadFile)
    x_rel = linspace(simulationParams.initPos(1),...
                     simulationParams.finalPos(1),...
                     simulationParams.collisionTime*simulationParams.sampleRate);
    y_rel = linspace(simulationParams.initPos(2),...
                     simulationParams.finalPos(2),...
                     simulationParams.collisionTime*simulationParams.sampleRate);
    z_rel = linspace(simulationParams.initPos(3),...
                     simulationParams.finalPos(3),...
                     simulationParams.collisionTime*simulationParams.sampleRate);
    relativePath = [x_rel', y_rel', z_rel'];
    clear x_rel y_rel z_rel
    timeVec = linspace(0,simulationParams.collisionTime, ...
              simulationParams.collisionTime*simulationParams.sampleRate);
end
%% SENSOR MODEL
%If sensor readings/time vec is saved, can load that to save time
if(loadFile)
    [file,path] = uigetfile('..\data\precomputed_sensors\*.mat',...
                            'Select a sensor reading file');
    load("" + path + file);
else
    sensorScenario = init_sensor_model(relativePath, timeVec, sensorParams,...
                                   targetParams);
    sensorReadings = sensor_model(sensorScenario, makeSensorPlot);
end

%% STATE ESTIMATION
%Determine how many sensor readings to use for batch LLS estimate
[offset, estimatorParams] = init_estimator(sensorReadings, estimatorParams);
estimatorParams.currentTime = timeVec(offset);

plotCount = 0;
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
 if ~isnan(estimate.corrState) %plot if count == 0, increment until reaches plotInterval, then reset to 0
    if plotCount == 0
        plotStruct = update_live_plot(plotStruct,estimate,recv);
        plotCount = plotCount + 1;
    elseif plotCount == plotStruct.interval
        plotCount = 0;
    else
        plotCount = plotCount+1;
    end
    
 end
 
 pause(real_time_delay)
end
%% CLEANUP
close_logging(plotStruct);
