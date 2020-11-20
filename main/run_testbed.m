%% HOUSEKEEPING
clear; clc; close all
importCast;

%% CONFIG
log_data = false;
scalingFactor = 222; %Distance scaling factor for testbed
makeSensorPlot = false;
loadFile = true;
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
simulationParams.initPos = [1.5,0.5,0]; %m, starting point of object
simulationParams.finalPos = [0,0,0]; %m, final point of object
simulationParams.collisionTime = 1.5; %s, time it takes to get from initial 
%to final position

%Estimator parameters
estimatorParams.llsSeeding = false;
estimatorParams.batchSamples = 0;
estimatorParams.sensorCovariance = [0.025^2,0;0,deg2rad(0.45)^2]; %Range, bearing
estimatorParams.qGain = 1; %Process noise gain for forward prediction

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
plotStruct.videoFig = figure;
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
lam = lam_vals(estimatorParams.sensorCovariance);
plotStruct.axis = [-.5 2 -1 1];
%% STATE ESTIMATION
%Determine how many sensor readings to use for batch LLS estimate
[offset, estimatorParams] = init_estimator(sensorReadings, estimatorParams);
estimatorParams.currentTime = timeVec(offset);
%  estimatorParams.filter.MotionModel = 'Custom';
% %  Process noise model used in matlab for 2D constant velocity
%  stepSize = 2.5004e-04;
%  estimatorParams.filter.ProcessNoise = [(stepSize^3)/3, (stepSize^2)/2,0,0;
%                                         (stepSize^2)/2, stepSize,0,0;
%                                         0,0,(stepSize^3)/3, (stepSize^2)/2;
%                                         0,0,(stepSize^2)/2, stepSize];
%  estimatorParams.filter.ProcessNoise = estimatorParams.filter.ProcessNoise./estimatorParams.qGain;
%  estimatorParams.filter.StateTransitionModel = [1,stepSize,0,0;
%                                     0,1,0,0;
%                                     0,0,1,stepSize;
%                                     0,0,0,1];
endOffset = 6000;
syms x
R_conv = @(r,theta)[- ((8983381526791141*limit(- (r*exp(- 800*r^2 + 1600*r*x - 800*x^2))/1600 - (x*exp(- 800*r^2 + 1600*r*x - 800*x^2))/1600 - (2^(1/2)*pi^(1/2)*erfi(-2^(1/2)*(r - x)*20i)*(r^2 + 1/1600)*1i)/80, x, -Inf))/562949953421312 - (8983381526791141*limit(- (r*exp(- 800*r^2 + 1600*r*x - 800*x^2))/1600 - (x*exp(- 800*r^2 + 1600*r*x - 800*x^2))/1600 - (2^(1/2)*pi^(1/2)*erfi(-2^(1/2)*(r - x)*20i)*(r^2 + 1/1600)*1i)/80, x, Inf))/562949953421312)*int((1787186969586535*exp(-(80000*(theta - x)^2)/pi^2)*cos(x)^2)/35184372088832, x, -Inf, Inf) - (80701143655892331564348236081881*r^2*pi*int((1787186969586535*exp(-(80000*(theta - x)^2)/pi^2)*cos(x))/35184372088832, x, -Inf, Inf)^2)/253530120045645880299340641075200,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           0, - ((8983381526791141*limit(- (r*exp(- 800*r^2 + 1600*r*x - 800*x^2))/1600 - (x*exp(- 800*r^2 + 1600*r*x - 800*x^2))/1600 - (2^(1/2)*pi^(1/2)*erfi(-2^(1/2)*(r - x)*20i)*(r^2 + 1/1600)*1i)/80, x, -Inf))/562949953421312 - (8983381526791141*limit(- (r*exp(- 800*r^2 + 1600*r*x - 800*x^2))/1600 - (x*exp(- 800*r^2 + 1600*r*x - 800*x^2))/1600 - (2^(1/2)*pi^(1/2)*erfi(-2^(1/2)*(r - x)*20i)*(r^2 + 1/1600)*1i)/80, x, Inf))/562949953421312)*int((1787186969586535*exp(-(80000*(theta - x)^2)/pi^2)*sin(x)^2)/35184372088832, x, -Inf, Inf) - (80701143655892331564348236081881*r^2*pi*int((1787186969586535*exp(-(80000*(theta - x)^2)/pi^2)*sin(x))/35184372088832, x, -Inf, Inf)^2)/253530120045645880299340641075200];
%% MAIN LOOP
tic
x_meas = [];
x_est = [];
y_meas = [];
tt = [];
sigx = [];
sigy = [];
sigvx = [];
sigvy = [];
y_est = [];
vx_est = [];
vy_est = [];
c_sigx = [];
c_sigy = [];
for i = offset : simulationParams.stepSize : endOffset
    % STATE ESTIMATION
 sensorReading = sensorReadings(i,:);
 if ~any(isnan(sensorReadings(i-1,:)))
     sensorReading(1,1:2) = [nan,nan];
 end
 time = timeVec(i);
 real_time_delay = 0;
 delay = 0;
 
 %Convert range-bearing to xy
 r = sensorReading(1);
 theta = sensorReading(2);
 sensorReading = [r*cos(theta), r*sin(theta)];
 


 %Account for conversion bias
 if(~any(isnan(sensorReading)))
     x_meas = [x_meas,sensorReading(1)];
y_meas = [y_meas,sensorReading(2)];
tt = [tt, time];
  estimatorParams.filter.MeasurementNoise = double(R_conv(r,theta));
 else
    continue
 end
 
 %Estimate the state
 
 [estimate, estimatorParams] = state_estimator(sensorReading, time,...
                                                      estimatorParams);

 collisionEstimate = collision_prediction(estimate, estimatorParams, collisionEstimate);
 c_sigx = [c_sigx, 2*sqrt(collisionEstimate.Ppred(1,1))];
 c_sigy = [c_sigy, 2*sqrt(collisionEstimate.Ppred(3,3))];
 
 % GUIDANCE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%make_maneuver and convert_2d currently have undefined behavior, both assume the second
%input is the chief orbit, which is not generated in the current code, the relative orbit
%has been substituted to enable full code execution, remove this comment block and
%bounding comments when fixed.
%  [maneuver,tAfter,stateAfter] = make_maneuver(collisionEstimate,relativeOrbit(i),...
%      collisionEstimate.collisionTime);
%  indForward = i+length(stateAfter);
%  tMove = timeVec(i:indForward);
%  [tMove,maneuverPos] = convert_2d(tMove,relativeOrbit(i:indForward,:),stateAfter);
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 maneuver = [0 0];
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
x_est = [x_est,estimate.predState(1)];
sigx = [sigx, 2*sqrt(estimate.Ppred(1,1))];
y_est = [y_est, estimate.predState(3)];
sigy = [sigy, 2*sqrt(estimate.Ppred(3,3))];
vx_est = [vx_est, estimate.predState(2)];
sigvx = [sigvx, 2*sqrt(estimate.Ppred(2,2))];
vy_est = [vy_est, estimate.predState(4)];
sigvy = [sigvy, 2*sqrt(estimate.Ppred(4,4))];
end
b_time = toc;
%% CLEANUP
close_logging(plotStruct);
t = timeVec(1:endOffset);
x_rel = 1.5 - 1*tt;
y_rel = 0.5 - (0.5/1.5)*tt;
vx = -1;
vy = -0.5/1.5;
figure
subplot(2,1,1)
hold on
plot(tt, x_est - x_rel)
plot(tt, sigx, '--k')
plot(tt, -sigx, '--k')
ylim([-0.5,0.5])
xlim([0, 1.2])
xlabel('Time [s]')
ylabel('X-Position Estimate Error [m/s]')
subplot(2,1,2)
hold on
plot(tt, y_est - y_rel)
plot(tt, sigy, '--k')
plot(tt, -sigy, '--k')
ylim([-0.5,0.5])
xlim([0, 1.2])
xlabel('Time [s]')
ylabel('Y-Position Estimate Error [m/s]')
legend('Estimate Error', '2\sigma Bound')

figure
subplot(2,1,1)
hold on
plot(tt, vx_est - vx)
plot(tt, sigvx, '--k')
plot(tt, -sigvx, '--k')
ylim([-0.5,0.5])
xlim([0, 1.2])
xlabel('Time [s]')
ylabel('X-Velocity Estimate Error [m/s]')
subplot(2,1,2)
hold on
plot(tt, vy_est - vy)
plot(tt, sigvy, '--k')
plot(tt, -sigvy, '--k')
ylim([-0.5,0.5])
xlim([0, 1.2])
xlabel('Time [s]')
ylabel('Y-Velocity Estimate Error [m/s]')
legend('Estimate Error', '2\sigma Bound')
