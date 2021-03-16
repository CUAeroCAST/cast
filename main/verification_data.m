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
v1 = 1.88; %initial speed (unsigned), m/s
v2 = 1.88;
v3 = 1.88;

r1 = [1.5,0]; %ramp position, m
r2 = [1.5,0.05];
r3 = [1.5,0.266];

ra1 = 0;
ra2 = 0.01693; %radians
ra3 = 0;

%% Generate Data
data1 = generate_test_data(sensorParams,targetParams,v1,r1,ra1);
data2 = generate_test_data(sensorParams,targetParams,v2,r2,ra2);
data3 = generate_test_data(sensorParams,targetParams,v3,r3,ra3);