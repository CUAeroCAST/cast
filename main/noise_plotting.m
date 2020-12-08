%% HOUSEKEEPING
clear; clc; close all
importCast;

%% VALUES TO CHANGE FOR TMT
simulationParams.initState = [1.5,-1,0.5,-0.5/1.5]; %m, starting state of object
simulationParams.collisionTime = 1.5; %s, time to simulate
simulationParams.Q = (1/10).*eye(4); %True value of process noise, used for adding noise to truth states

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
timeVec = linspace(0,simulationParams.collisionTime, ...
        simulationParams.collisionTime*simulationParams.sampleRate);
process_noise = mvnrnd([0,0,0,0], simulationParams.Q, length(timeVec))';
x_true(:,1) = simulationParams.initState';
for i = 1:(length(timeVec) - 1)
    tspan = [timeVec(i), timeVec(i+1)];
    [t_ode, x_ode] = ode45(@(t,x) linear_ode(t,x,process_noise(:,i)), tspan, x_true(:,i));
    x_true(:,i+1) = x_ode(end,:)';
end
%relative path is just x,y,z
relativePath = [x_true(1,:)', x_true(3,:)', zeros(length(timeVec),1)];
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
y = nan(length(timeVec),2);
x_p(:,1) = simulationParams.initState';
x_c(:,1) = nan(4,1);
for i = 2 : simulationParams.stepSize : length(timeVec)
    % STATE ESTIMATION
    sensorReading = sensorReadings(i,:);
    time = timeVec(i);
    if(~any(isnan(sensorReading)))
        %Convert range-bearing to xy
        mu = conv_meas_bias(lam, sensorReading);
        sensorReading = meas2cart(sensorReading, mu);
        y(i,:) = sensorReading;
        %Account for conversion bias
        if(~any(isnan(sensorReading)))
            R_conv = get_conv_cov(estimatorParams.sensorCovariance, lam, sensorReading);
            estimatorParams.filter.MeasurementNoise = R_conv;
        end
    else
        y(i,:) = nan(1,2);
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
figure;
sgtitle('x and y States vs Time','fontsize',24)
subplot(2,1,1)
hold on
scatter(timeVec,y(:,1),'o','LineWidth',3)
plot(timeVec,x_true(1,:),'LineWidth',3)
title('x vs Time','Interpreter','latex')
xlabel('Time [s]','fontsize',22,'Interpreter','latex')
ylabel('x [m]','fontsize',24,'Interpreter','latex')
set(gca, 'FontSize', 20)
legend("Measurement","True State",'fontsize',14,"location",...
    "best",'Interpreter','latex')
grid minor;


subplot(2,1,2)
hold on
scatter(timeVec,y(:,2),'o','LineWidth',3)
plot(timeVec,x_true(3,:),'LineWidth',3)
title('y vs Time','Interpreter','latex')
xlabel('Time [s]','fontsize',22,'Interpreter','latex')
ylabel('y [m]','fontsize',24,'Interpreter','latex')
set(gca, 'FontSize', 20)
legend("Measurement","True State",'fontsize',14,"location",...
    "best",'Interpreter','latex')
grid minor;