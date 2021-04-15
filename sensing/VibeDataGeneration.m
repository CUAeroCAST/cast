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

timeVec = linspace(0, 2, 2*sensorParams.samplingRate);
%% SENSOR MODEL
scenario = trackingScenario('UpdateRate',sensorParams.samplingRate);
scenario.StopTime = timeVec(end);
ego = platform(scenario,'Trajectory', ...
                kinematicTrajectory('Position',[0 0 0],...
                'Velocity', [0.1, 0.1, 0],...
                'AngularVelocitySource', 'Property',...
                'AngularVelocity',[0 0 sensorParams.scanRate*2*pi]));
% Stationary object
target = platform(scenario);
target.Mesh = targetParams.Mesh;
target.Dimensions = targetParams.Dimensions;
sensorOpts = {'UpdateRate',sensorParams.samplingRate, ...
                 'MountingLocation',[0,0,0], ...
                 'MountingAngles',[0,0,0], ...
                 'MaxRange',sensorParams.maxRange, ...
                 'RangeAccuracy',sensorParams.rangeAccuracy, ...
                 'AzimuthResolution',sensorParams.beamDivergence, ...
                 'ElevationResolution',sensorParams.beamDivergence,...
                 'AzimuthLimits',sensorParams.beamLimits,  ...
                 'ElevationLimits',sensorParams.beamLimits, ...
                 'HasNoise',true, ...
                 'HasINS',true,...
                 'DetectionCoordinates','Sensor'};
sensor = monostaticLidarSensor(1,sensorOpts{:});
ego.Sensors = sensor;
%Collect sensor data
n_steps = floor(scenario.UpdateRate*(scenario.StopTime - scenario.SimulationTime));
%Store sensor readings in cell matrix as we don't know the # of channels
sensorReadings = zeros(n_steps,2);
xy_loc = zeros(n_steps,3);
for i = 1:n_steps 
    advance(scenario);
    tgtmeshes = targetMeshes(ego);
    temp = sensor(tgtmeshes,pose(ego),scenario.SimulationTime);
    %Take the middle reading (should be center beam)
    sensorReadings(i,1) = temp(median(1:length(temp(:,1))),1);
    sensorReadings(i,2) = deg2rad(ego.Orientation(1));
    xy_loc(i,:) = ego.Position;
end
notValid = isnan(sensorReadings(:,1));
xy_loc(notValid,:) = [];
sensorReadings(notValid,:) = [];

x = xy_loc(:,1) + sensorReadings(:,1).*cos(sensorReadings(:,2));
y = xy_loc(:,2) + sensorReadings(:,1).*sin(sensorReadings(:,2));

figure
hold on
plot(x,y,'x')
axis equal

% 2 sig bound
a=2*std(x); % horizontal radius
b=2*std(y); % vertical radius
x0=mean(x); % x0,y0 ellipse centre coordinates
y0=mean(y);
t=-pi:0.01:pi;
x_2sig=x0+a*cos(t);
y_2sig=y0+b*sin(t);
plot(x_2sig,y_2sig, 'r')

%actual ball
a=25e-3; % horizontal radius
b=25e-3; % vertical radius
x0=0; % x0,y0 ellipse centre coordinates
y0=0;
t=-pi:0.01:pi;
x_ball=x0+a*cos(t);
y_ball=y0+b*sin(t);
plot(x_ball,y_ball, 'g')
xlabel('X Position [m]')
ylabel('Y Position [m]')
legend('Simulated Data','2\sigma Bound', 'Actual Ball')
title('Expected Vibration Response')

%Compute number of points within circle
radius = 0.025;
x_ball = 0;
y_ball = 0;
x0 = x - x_ball;
y0 = y - y_ball;
n_within = sum(x0.^2 + y0.^2 - radius^2 < 0);
n_total = length(x);
pct_within = n_within/n_total*100;
