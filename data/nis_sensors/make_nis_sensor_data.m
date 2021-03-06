%% HOUSEKEEPING
clear; clc; close all
importCast;

%% CONFIG

log_data = false;
hideFig = false;
scalingFactor = 222; %Distance scaling factor for testbed
makeSensorPlot = false;
loadFile = false;
isOrbitalScenario = false;
isTestbedScenario = true;

datapath = open_logging(log_data);

gmatParams = struct;
guidanceParams = struct;
plotStruct = struct;
recv = struct;

%Simulation parameters
simulationParams.stepSize = 1;
simulationParams.sampleRate = 4e3;
simulationParams.scalingFactor = 1; %Distance scaling factor for testbed
simulationParams.initPos = [1.5,0.1061,0]; %m, starting point of object
simulationParams.finalPos = [0,.1061,0]; %m, final point of object
simulationParams.collisionTime = 1.5; %s, time it takes to get from initial 
%to final position

%Estimator parameters
estimatorParams.llsSeeding = false;
estimatorParams.batchSamples = 0;
estimatorParams.sensorCovariance = [0.025^2,0;0,deg2rad(0.45)^2]; %Range, bearing
estimatorParams.qGain = 1; %Process noise gain for forward prediction
estimatorParams.initState = [1.5;-1;0;0]; %Constant x-vel init state

%Sensor parameters
sensorParams.samplingRate = 4e3;
sensorParams.maxRange = 4e3;
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
if hideFig
    plotStruct.videoFig = figure('visible','off');
else
    plotStruct.videoFig = figure;    
end
plotStruct.collisionFlag = 0;   %flag for if covariance has intersected with gantry pos
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
plotStruct.interval = 50; %how often the plot is updated... if equals 0, then it plots every iteration
plotCount = 0; %initial count value for iterating inside main loop
plotCorrCount = 50; %starting value for corr state plot count

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
%Save parameter structs
if log_data
 log_struct(gmatParams, [datapath, filesep, 'gmatParams'])
 log_struct(estimatorParams, [datapath, filesep, 'estimatorParams'])
 log_struct(guidanceParams, [datapath, filesep, 'guidanceParams'])
end
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
if (isTestbedScenario)
    % Initialize very large collision estimation, represents no knowledge of
    % collision before test starts
    collisionEstimate.Ppred = [10 0 0 0 0 0;0 0 0 0 0 0;0 0 10 0 0 0;...
    0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0];%x and y radius, initially, 10x10
    collisionEstimate.predState = [0 0 0 0 0 0];
    collisionEstimate.collisionTime = 1; %dummy value
    if ~loadFile
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
end

%% SENSOR MODEL
% If sensor readings/time vec is saved, can load that to save time
xx = 1;
for i = 1:1000
    sensorScenario = init_sensor_model(relativePath, timeVec, sensorParams,...
        targetParams);
    sensorReadings = sensor_model(sensorScenario, makeSensorPlot);
    fileNum = num2str(i);
    filename = ['nis_scenario_' fileNum '.mat'];
    saveVars.relativePath = relativePath;
    saveVars.timeVec = timeVec;
    saveVars.sensorReadings = sensorReadings;
    save(filename,'relativePath','timeVec','sensorReadings')
end

%Constants for converting measurement to cartesian
lam = lam_vals(estimatorParams.sensorCovariance);
plotStruct.axis = [-.5 2 -1.25 1.25];