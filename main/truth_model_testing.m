%% HOUSEKEEPING
clear; clc; close all
importCast;

%% VALUES TO CHANGE FOR TMT
simulationParams.initPos = [1.5,0.5,0]; %m, starting point of object
simulationParams.finalPos = [0,0,0]; %m, final point of object
simulationParams.collisionTime = 1.5; %s, time it takes to get from initial 
simulationParams.Q = (1/100).*eye(4); %True value of process noise, used for adding noise to truth states

estimatorParams.sensorCovariance = [0.025^2,0;0,deg2rad(0.45)^2]; %Range, bearing
estimatorParams.qGain = 1; %Process noise gain for forward prediction
estimatorParams.initState = [1.5;-1;0;0]; %Constant x-vel init state

%% OTHER CONFIG (Don't Tweak)
%Simulation parameters (for sensor model)
simulationParams.stepSize = 1;
simulationParams.sampleRate = 4e3;
simulationParams.scalingFactor = 1; %Distance scaling factor for testbed
%to final position

%Estimator parameters
estimatorParams.llsSeeding = false;
estimatorParams.batchSamples = 0;

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

%% TESTBED COLLISION
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
timeVec = linspace(0,simulationParams.collisionTime, ...
        simulationParams.collisionTime*simulationParams.sampleRate);

%% STASTICAL STUFF
simulationParams.xvel = (simulationParams.finalPos(1) - simulationParams.initPos(1))/simulationParams.collisionTime;
simulationParams.yvel = (simulationParams.finalPos(2) - simulationParams.initPos(2))/simulationParams.collisionTime;
x_true = [x_rel;
          simulationParams.xvel*ones(1,length(timeVec));
          y_rel;
          simulationParams.yvel*ones(1,length(timeVec))];
process_noise = mvnrnd([0,0,0,0], simulationParams.Q, length(timeVec))';
x_true = x_true + process_noise;
%add x-dir process noise to relative path
relativePath(:,1) = relativePath(:,1) + process_noise(1,:)';
%add y-dire process noise to relative path
relativePath(:,2) = relativePath(:,3) + process_noise(3,:)';
%% SENSOR MODEL
%If sensor readings/time vec is saved, can load that to save time
sensorScenario = init_sensor_model(relativePath, timeVec, sensorParams,...
                                   targetParams);
sensorReadings = sensor_model(sensorScenario, false);

%Constants for converting measurement to cartesian
lam = lam_vals(estimatorParams.sensorCovariance);
%% STATE ESTIMATION
%Determine how many sensor readings to use for batch LLS estimate
[offset, estimatorParams] = init_estimator(sensorReadings, estimatorParams);
estimatorParams.currentTime = timeVec(offset);

%% MAIN LOOP
for i = 2 : simulationParams.stepSize : length(timeVec)
    % STATE ESTIMATION
    sensorReading = sensorReadings(i,:);
    time = timeVec(i);
    if(~any(isnan(sensorReading)))
        %Convert range-bearing to xy
        mu = conv_meas_bias(lam, sensorReading);
        sensorReading = meas2cart(sensorReading, mu);

        %Account for conversion bias
        if(~any(isnan(sensorReading)))
            R_conv = get_conv_cov(estimatorParams.sensorCovariance, lam, sensorReading);
            estimatorParams.filter.MeasurementNoise = R_conv;
        end
    end
    %Estimate the state
    [estimate, estimatorParams] = state_estimator(sensorReading, time,...
                                                  estimatorParams);
    %Save off important values for truth model testing
    xp(:,i) = estimate.predState;
    xc(:,i) = estimate.corrState;
    
    Pp{i} = estimate.Ppred;
    Pc{i} = estimate.Pcorr;
end
%Use these variables for each trial
%xtrue, timeVec, xp,xc,Pp,Pc, sensorReadings
%test_NIS( eyk, yk_minus, Pk_minus,H,R, tvec, N, alpha )