%% HOUSEKEEPING
clear; clc; close all
importCast;
open_logging();

%% CONFIG
gmatParams = struct;
estimatorParams = struct;
PlotStruct = struct;
recv = struct;
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

%[chiefOrbit, deputyOrbit, timeVec] = make_gmat_orbits(gmatParams);

%relativeOrbit = chiefOrbit - deputyOrbit;
%Temp creation of relative orbit for init_sensor_model
timeSteps = 100;
xrel = linspace(.1,0,timeSteps)';
%yrel = linspace(2*10,0,timeSteps)';
yrel = zeros(timeSteps,1);
zrel = zeros(timeSteps,1);
relativeOrbit = [xrel yrel zrel];
timeVec = 0:timeSteps-1;

%% SENSOR MODEL

sensorScenario = init_sensor_model(relativeOrbit, timeVec, sensorParams,...
                                   targetParams);
sensorReadings = sensor_model(sensorScenario);

%% STATE ESTIMATION

[offset, estimatorParams] = init_estimator(sensorReadings, estimatorParams);
estimatorParams.currentTime = timeVec(offset);

%% LIVE PLOT INITIALIZATION
plotStruct.filename = '2D_collision_avoid';
plotStruct.vobj = VideoWriter(plotStruct.filename, 'MPEG-4');
plotStruct.vobj.Quality = 100;
open(plotStruct.vobj);
plotStruct.videoFig = figure;
plotStruct.axis = [-1*.01*max(xrel),max(xrel),-1*.01*max(xrel),max(xrel)];
plotStruct.plots2Delete = 0;
plotStruct.collisionFlag = 0;   %flag for if covariance has intersected with gantry pos

%% MOTOR INPUT DUMMY SETUP
%stationary for half of time, +y movement for the rest
firstHalf = zeros(timeSteps/2,1);
secondHalf = linspace(0,.1,timeSteps/2)';
motorInput = [zeros(timeSteps,1) [firstHalf;secondHalf]];

%% MAIN LOOP
for i = offset : estimatorParams.stepSize : length(timeVec)
 % STATE ESTIMATION
 sensorReading = sensorReadings{i};
 time = timeVec(i);
 real_time_delay = 0.01;
 
 
 [estimate, estimatorParams] = state_estimator(sensorReading, time,...
                                                      estimatorParams);

 % GUIDANCE

 %[maneuver, delay] = make_maneuver(estimate, guidanceParams);
 
 recv = dummy_run_io(motorInput(i,:), 0, 1, real_time_delay);
 
 % Visualization
 
 plotStruct = update_live_plot(plotStruct,estimate,recv,i);
 
 pause(real_time_delay)

end
%% CLEANUP

close_logging(plotStruct);