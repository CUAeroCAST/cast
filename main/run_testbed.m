%% HOUSEKEEPING
clearvars -except burnTable accelTable timeTable positionTable aChief chiefOrbit tChief; 
clc; close all
importCast;
addpath('../../../LookupTables')    %This needs to change to where people keep their tables
% load('burnTable.mat');
% load('positionTable.mat');
% load('timeTable.mat');
% load('accelTable.mat');
% load('aChief.mat');
% load('chiefOrbit.mat');
% load('tChief.mat');

%% CONFIG

log_data = false;
hideFig = true;
scalingFactor = 222; %Distance scaling factor for testbed
makeSensorPlot = false;
loadFile = true;
isOrbitalScenario = false;
isTestbedScenario = true;

global datapath;
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

%Sensor parameters
sensorParams.samplingRate = 4e3;
sensorParams.maxRange = 4e3;
sensorParams.beamDivergence = 0.9; %deg
sensorParams.rangeAccuracy = 0.025; %m
sensorParams.beamLimits = [-1.35,1.35];
sensorParams.sensorType = 'Lidar';
sensorParams.scanRate = 10; %Hz
sensorParams.readsize = 5;
sensorParams.scanMode = "standard";
sensorParams.portstr = "COM3";
sensorParams.sensorObj = serial_Sensor(sensorParams);

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
    plotStruct.videoFig = figure('visible', 'off');
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

%Constants for converting measurement to cartesian
plotStruct.axis = [-.5 2 -1.25 1.25];
%% STATE ESTIMATION
%Estimator parameters
estimatorParams.currentTime = timeVec(1);
estimatorParams.filter.MeasurementModel = @(x,xs,y,ys)...
    [(2*x - 2*xs)/(2*((x - xs)^2 + (y - ys)^2)^(1/2)),...
    (2*y - 2*ys)/(2*((x - xs)^2 + (y - ys)^2)^(1/2)), 0, 0;
    -(y - ys)/((x - xs)^2*((y - ys)^2/(x - xs)^2 + 1)),...
    1/((x - xs)*((y - ys)^2/(x - xs)^2 + 1)), 0, 0]; %Range bearing model
%Initial state
estimatorParams.filter.State = [1.5;0;-1;0]; %Constant x-vel init state (x,y,vx,vy)
%Measurement noise (sensor Covariance)
estimatorParams.filter.MeasurementNoise = [0.025^2,0;0,deg2rad(0.45)^2]; %Range, bearing
estimatorParams.filter.ProcessNoise = @(dt) 0.01*eye(4); %constant process noise
estimatorParams.filter.StateCovariance = eye(4); %Initial estimate covariance
estimatorParams.filter.STM = @(dt) eye(4) + [0,0,1,0;0,0,0,1;0,0,0,0;0,0,0,0]*dt;
%% MAIN LOOP
moving = 0;
try
 while true
  % STATE ESTIMATION
  %this is ready to be swapped with sensor grabber
  sensorReading(:,1) = sensorReadings(i,:); %Needs to be column vector of range-bearing
  time = timeVec(i);
  real_time_delay = 0;
  delay = 0;
  if(~any(isnan(sensorReading)))

      %Estimate the state
      [estimate, estimatorParams] = state_estimator(sensorReading, time,...
                                                           estimatorParams);
      collisionEstimate = collision_prediction(estimate, estimatorParams, collisionEstimate);
  else
     estimate.predState = nan(4,1);
     estimate.Ppred = nan(4,4);
     estimate.corrState = nan(4,1);
     estimate.Pcorr = nan(4,4);
  end

  % GUIDANCE
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %make_maneuver and convert_2d currently have undefined behavior, both assume the second
 %input is the chief orbit, which is not generated in the current code, the relative orbit
 %has been substituted to enable full code execution, remove this comment block and
 %bounding comments when fixed.
 if ~moving
     maneuver = [0 0];
     recv = run_io(maneuver, delay);
 end
 if ~moving
     chiefState = [0;0;7578;7.25256299066873;0;0];
     muE = 398600;
     tstep = timeVec(2)-timeVec(1);
     [maneuver,tAfter,stateAfter] = make_maneuver(collisionEstimate,chiefState,...
         1.5-time,length(timeVec)-i);
     if maneuver
         [tChief, chiefOrbit] = ode45(@(t, y) orbit_prop(t, y, muE), tAfter, chiefState);
         [tAfter,maneuverPos] = convert_2d(tAfter,chiefOrbit,stateAfter(:,1:6));
         divideTimes = tAfter(end)/(timeVec(end)-time);
         tAfter = tAfter/divideTimes;
         tAfter = tAfter+time;
         reduceLength = length(tAfter)/(length(timeVec)-i);
         tIndex = 1;
         tMove = [];
         movementPos = [];
         for j = 1:length(timeVec)-i
             tMove = [tMove; tAfter(tIndex)];
             movementPos = [movementPos maneuverPos(1:2,tIndex)];
             tIndex = tIndex+floor(reduceLength);
         end
         moving = 1;
     end
 end
<<<<<<< HEAD
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  maneuver = [0 0];
  %recv = run_io(maneuver, delay);

  % Visualization
 %  if ~(isnan(estimate.corrState(1))) && (plotCorrCount >= plotStruct.interval)%plot if count == 0, increment until reaches plotInterval, then reset to 0
 %      plotStruct = update_live_plot(plotStruct,estimate,collisionEstimate,recv);
 %      plotCorrCount = 0; %reset count so at least plotStruct.interval steps have to pass before another plot update
 %      plotCount = 1;%reset count so plotCount.interval iterations need to pass before another update
 %  elseif plotCount == 0 %plotCount has reached and count has been set to 0, update plot
 %      plotCount = plotCount + 1; 
 %      plotStruct = update_live_plot(plotStruct,estimate,collisionEstimate,recv);
 %  elseif plotCount == plotStruct.interval 
 %      plotCount = 0; %reset to 0 so that plot can update next iteration
 %  else
 %      plotCount = plotCount+1; 
 %  end
 % plotCorrCount = plotCorrCount + 1; %increment no matter what
 end
catch ME
=======

 % GUIDANCE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%make_maneuver and convert_2d currently have undefined behavior, both assume the second
%input is the chief orbit, which is not generated in the current code, the relative orbit
%has been substituted to enable full code execution, remove this comment block and
%bounding comments when fixed.
if ~moving
    maneuver = [0 0];
    recv = run_io(maneuver, delay);
end
if ~moving
    chiefState = [0;0;7578;7.25256299066873;0;0];
    muE = 398600;
    tstep = timeVec(2)-timeVec(1);
    maneuver = make_maneuver(collisionEstimate,chiefState,...
        1.5-time,length(timeVec)-i,burnTable,positionTable,timeTable);
    if maneuver
        direction = round(atand(maneuver(2)/maneuver(1)));
        tAfter = timeTable{maneuver(3),direction};
        accel = accelTable{maneuver(3),direction};
        stateAfter = burnTable{maneuver(3),direction};
        [tAfter,maneuverPos] = convert_2d(tAfter,chiefOrbit,stateAfter(:,1:6));
        [tAfter,maneuverAcc] = convert_2d_accel(tAfter,chiefOrbit,stateAfter(:,1:6),aChief,accel);
        divideTimes = tAfter(end)/(timeVec(end)-time);
        tAfter = tAfter/divideTimes;
        tAfter = tAfter+time;
        reduceLength = length(tAfter)/(length(timeVec)-i);
        tIndex = 1;
        tMove = [];
        movementPos = [];
        for j = 1:length(timeVec)-i
            tMove = [tMove; tAfter(tIndex)];
            movementPos = [movementPos maneuverPos(1:2,tIndex)];
            tIndex = tIndex+floor(reduceLength);
        end
        moving = 1;
    end
end
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 maneuver = [0 0];
 %recv = run_io(maneuver, delay);
 
 % Visualization
 if moving
     plotIndex = find(time<tMove,1);
     if isempty(plotIndex)
         recv.state = prevState;
     else
         prevState = movementPos(:,plotIndex);
         recv.state = movementPos(:,plotIndex);
     end
 end
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

%Additonal KF Plotting
% t = [t, time];
% x = [x, estimate.predState(1)];
% y = [y, estimate.predState(3)];
% vx = [vx, estimate.predState(2)];
% vy = [vy, estimate.predState(4)];
% sigx = [sigx, 2*sqrt(estimate.Pcorr(1,1))];
% sigy = [sigy, 2*sqrt(estimate.Pcorr(3,3))];
% sigvx = [sigvx, 2*sqrt(estimate.Pcorr(2,2))];
% sigvy = [sigvy, 2*sqrt(estimate.Pcorr(4,4))];
% pause(real_time_delay)
end
>>>>>>> d39809b5dc6146436e46a9e06fec40c813c48c14
%% CLEANUP
 close_logging(plotStruct);
 rethrow(ME)
end
