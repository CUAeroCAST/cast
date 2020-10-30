%% HOUSEKEEPING
clear; clc; close all
importCast;
open_logging();

%% CONFIG
scalingFactor = 222; %Distance scaling factor for testbed

gmatParams = struct;
estimatorParams = struct;
PlotStruct = struct;
recv = struct;
 estimatorParams.stepSize = 1;
 estimatorParams.initState = [0,0,0,0,0,0];
guidanceParams = struct;

%Sensor parameters
sensorParams.samplingRate = 1e3;
sensorParams.maxRange = 4e3;
sensorParams.beamDivergence = 0.5; %deg
sensorParams.rangeAccuracy = 0.025; %m
sensorParams.beamLimits = [-0.75,0.75];
sensorParams.sensorType = 'Lidar';

%Target parameters
targetParams.Mesh = extendedObjectMesh('sphere');
targetParams.Dimensions.Length = 4.5e-3; 
targetParams.Dimensions.Width = 4.5e-3;
targetParams.Dimensions.Height = 4.5e-3;
targetParams.Dimensions.OriginOffset = [0,0,0];

%Avoidance Parameters
canAvoid = false;
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
timeVec = linspace(0,n/1000,n)';
%% SENSOR MODEL

sensorScenario = init_sensor_model(relativeOrbit, timeVec, sensorParams,...
                                   targetParams);
sensorReadings = sensor_model(sensorScenario);

%% STATE ESTIMATION

[offset, estimatorParams] = init_estimator(sensorReadings, estimatorParams);
estimatorParams.currentTime = timeVec(offset);

%% LIVE PLOT INITIALIZATION
filename = '2D_collision_avoid';
vobj = VideoWriter(filename, 'MPEG-4');
vobj.Quality = 100;
open(vobj);
videoFig = figure;
axis = [-1,20,-1,20];

collisionFlag = 0;   %flag for if covariance has intersected with gantry pos

%% MAIN LOOP
for i = offset : estimatorParams.stepSize : length(timeVec)
 % STATE ESTIMATION
 sensorReading = sensorReadings{i};
 time = timeVec(i);
 real_time_delay = 0;
 
 [estimate, estimatorParams] = state_estimator(sensorReading, time,...
                                                      estimatorParams);
 
 %This needs to be set when the covariance elipse is within bounds, only
 if(estimate.corrState(2)>0)
  collisionTime = estimate.corrState(1)/estimate.corrState(2) + ...
                  estimatorParams.currentTime;
  %need to calculate it when we can avoid
  collisionEstimate = desync_predict(collisionTime, estimatorParams); 
 end
 
 % GUIDANCE

 [maneuver, delay] = make_maneuver(estimate, guidanceParams);
 
 recv = run_io(maneuver, delay);
 
 % Visualization
 
 %[PlotStruct,collisionFlag] = update_live_plot(PlotStruct,estimate,recv,vobj,axis,collisionFlag,i);
 
 pause(real_time_delay)

end
%% CLEANUP
close(vobj);
close(videoFig)
close_logging();
