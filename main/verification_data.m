%% Housekeeping
close all
clear
clc
%% Parameters
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

%Scenario parameters
v1 = 1.5; %initial speed (unsigned), m/s
v2 = 1.5;
v3 = 1.5;

r1 = [1.5,0]; %ramp position, m
r2 = [1.5,0.05];
r3 = [1.5,0.266];

ra1 = 0;
ra2 = 0.01693; %radians
ra3 = 0;

%% Generate Data
for i = 1:100
    data = generate_test_data(sensorParams,targetParams,v1,r1,ra1);
    fileNum = num2str(i);
    fileName = ['consistancyTest' fileNum '.mat'];
    save(fileName,'data')
end
