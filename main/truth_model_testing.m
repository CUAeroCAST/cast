%% HOUSEKEEPING
clear; clc; close all
importCast;

%% CONFIG

log_data = false;
hideFig = false;
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
simulationParams.initPos = [1.5,0.1061,0]; %m, starting point of object
simulationParams.finalPos = [0,.1061,0]; %m, final point of object
simulationParams.collisionTime = 1.5; %s, time it takes to get from initial 
%to final position

%Estimator parameters
estimatorParams.llsSeeding = false;
estimatorParams.batchSamples = 0;
estimatorParams.sensorCovariance = [0.025^2,0;0,deg2rad(0.45)^2]; %Range, bearing
estimatorParams.qGain = 1; %Process noise gain for forward prediction
estimatorParams.initState = [1.5;0;-1;0]; %Constant x-vel init state

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
% plotStruct.filename = [datapath, filesep, '2D_collision_avoid'];
% plotStruct.vobj = VideoWriter(plotStruct.filename, 'MPEG-4');
% plotStruct.vobj.Quality = 100;
% open(plotStruct.vobj);
% if hideFig
%     plotStruct.videoFig = figure('visible','off');
% else
%     plotStruct.videoFig = figure;    
% end
% plotStruct.collisionFlag = 0;   %flag for if covariance has intersected with gantry pos
% set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
% plotStruct.interval = 50; %how often the plot is updated... if equals 0, then it plots every iteration
% plotCount = 0; %initial count value for iterating inside main loop
% plotCorrCount = 50; %starting value for corr state plot count

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
plotStruct.axis = [-.5 2 -1.25 1.25];
%% NIS Loop
nisSum = zeros(1,14);
N = 1;

%Determine how many sensor readings to use for batch LLS estimate
[offset, estimatorParams] = init_estimator(sensorReadings, estimatorParams);
estimatorParams.currentTime = timeVec(offset);
estimate.corrState = estimatorParams.initState;
estimate.Pcorr = diag([10 10 10 10]);
%Additional KF Plotting
x = [];
sigx = [];
y = [];
sigy = [];
vx = [];
sigvx = [];
vy = [];
sigvy = [];
t = [];
%% STATE ESTIMATION
%Determine how many sensor readings to use for batch LLS estimate
[offset, estimatorParams] = init_estimator(sensorReadings, estimatorParams);
estimatorParams.currentTime = timeVec(offset);
Hvec = {};
Pk_minus = {};
yk = [];
ykhat = [];
sensorRecordings = [];
timeRecordings = [];
performEstimation = false;
%% MAIN LOOP
for i = 2 : simulationParams.stepSize : length(timeVec)
    % STATE ESTIMATION
    sensorReading = sensorReadings(i,:);
    time = timeVec(i);
    deltat = timeVec(i)-timeVec(i-1);
    real_time_delay = 0;
    delay = 0;
    if(~any(isnan(sensorReading)))
        sensorRecordings = [sensorRecordings;sensorReading];
        timeRecordings = [timeRecordings time];
        performEstimation=true;
    elseif(performEstimation)
        %Estimate the state
        meanReading = mean(sensorRecordings);
        meanTime = mean(timeRecordings);
        Q = diag([.05 .05]);
        R = diag([deg2rad(1)  .01]);
        [estimate,estimatorParams,H,ymeas,ypred] = ekf(meanReading, timeVec(i-1), estimatorParams, 0, 0);
        Hvec{end+1} = H;
        Pk_minus{end+1} = estimate.Ppred;
        yk = [yk ymeas];
        ykhat = [ykhat ypred];
        %Additonal KF Plotting
        t = [t, time];
        x = [x, estimate.predState(1)];
        y = [y, estimate.predState(2)];
        vx = [vx, estimate.predState(3)];
        vy = [vy, estimate.predState(4)];
        sigx = [sigx, 2*sqrt(estimate.Pcorr(1,1))];
        sigy = [sigy, 2*sqrt(estimate.Pcorr(3,3))];
        sigvx = [sigvx, 2*sqrt(estimate.Pcorr(2,2))];
        sigvy = [sigvy, 2*sqrt(estimate.Pcorr(4,4))];
        pause(real_time_delay)
        performEstimation = false;
        sensorRecordings = [];
        timeRecordings = [];
    else
        estimate.predState = nan(4,1);
        estimate.Ppred = nan(4,4);
        estimate.corrState = nan(4,1);
        estimate.Pcorr = nan(4,4);
    end
    
    %Save off important values for truth model testing
    xp(:,i) = estimate.predState;
    xc(:,i) = estimate.corrState;
    
    Pp{i} = estimate.Ppred;
    Pc{i} = estimate.Pcorr;
    
    
    
    
end
nees = NaN(1, length(t));
nis = NaN(1, length(t));
S = cell(1,length(t));
R = estimatorParams.sensorCovariance;
ey = yk-ykhat;
for k = 1:length(t)
    %     nees(k) = ex(:, k)'*inv(Pk{k})*ex(:, k);
    S{k} = Hvec{k} * Pk_minus{k} * Hvec{k}' + R;
    nis(k) = ey(:, k)'*inv(S{k})*ey(:, k);
end
nisSum = nis+nisSum;


%Use these variables for each trial
%xtrue, timeVec, xp,xc,Pp,Pc, sensorReadings
alpha = 0.05;
test_NIS( nisSum, t, N, alpha )