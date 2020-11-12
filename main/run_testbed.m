%% HOUSEKEEPING
clear; clc; close all
importCast;
datapath = open_logging();

%% CONFIG
makeSensorPlot = false;
loadFile = false;
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
simulationParams.initPos = [2,0,0]; %m, starting point of object
simulationParams.finalPos = [0,0,0]; %m, final point of object
simulationParams.collisionTime = 2; %s, time it takes to get from initial 
%to final position

%Estimator parameters
estimatorParams.llsSeeding = false;
estimatorParams.batchSamples = 0;
estimatorParams.sensorCovariance = [0.025^2,0;0,0.45^2,];

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
plotCount = 0; %initial count value for iterating inside main loop
plotCorrCount = 50; %starting value for corr state plot count

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
if (isTestbedScenario)
    % Initialize very large collision estimation, represents no knowledge of
    % collision before test starts
    collisionEstimate.Ppred = [10 0 0 0 0 0;0 0 0 0 0 0;0 0 10 0 0 0;...
    0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0];%x and y radius, initially, 10x10
    collisionEstimate.predState = [0 0 0 0 0 0];
    collisionEstimate.collisionTime = 100; %dummy value
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

%Convert measurement to cartesian
lam = lam_vals(estimatorParams.sensorCovariance);
for i=1:length(sensorReadings)
    mu = conv_meas_bias(lam, sensorReadings(i,:));
    sensorReadings(i,:) = meas2cart(sensorReadings(i,:), mu);
end
plotStruct.axis = [-.5 2 -1 1];
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
 
 %Account for conversion bias
 R_conv = get_conv_cov(estimatorParams.sensorCovariance, lam, sensorReading);
 if(~any(isnan(R_conv)))
  estimatorParams.filter.MeasurementNoise = R_conv;
 end
 
 %Estimate the state
 [estimate, estimatorParams] = state_estimator(sensorReading, time,...
                                                      estimatorParams);

 %Forward prediction when collision time can be predicted
 if(estimate.predState(2)<0)
  collisionEstimate.collisionTime = -estimate.predState(1)/estimate.predState(2) + ...
                  estimatorParams.currentTime;
  %need to calculate it when we can avoid
  collisionEstimate = desync_predict(collisionEstimate.collisionTime, estimatorParams); 
 end
 
 % GUIDANCE

 [maneuver, delay] = make_maneuver(estimate, guidanceParams);
 
 recv = run_io(maneuver, delay);
 
 % Visualization
 if ~(isnan(estimate.corrState(1))) && (plotCorrCount >= plotStruct.interval)%plot if count == 0, increment until reaches plotInterval, then reset to 0
     plotStruct = update_live_plot(plotStruct,estimate,collisionEstimate,recv);
     plotCorrCount = 0; %reset count so at least plotStruct.interval steps have to pass before another plot update
     plotCount = 1;%reset count so plotCount.interval iterations need to pass before another update
 elseif plotCount == 0 %plotCount has reached and count has been set to 0, update plot
     plotCount = plotCount + 1; 
     plotStruct = update_live_plot(plotStruct,estimate,collisionEstimate,recv);
 elseif plotCount == plotStruct.interval 
     plotCount = 0; %reset to 0 so that plot can update next iteration
 else
     plotCount = plotCount+1; 
 end
plotCorrCount = plotCorrCount + 1; %increment no matter what

pause(real_time_delay)
end
%% CLEANUP
close_logging(plotStruct);
