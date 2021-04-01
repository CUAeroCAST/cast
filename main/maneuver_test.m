clearvars -except burnTable accelTable timeTable positionTable aChief chiefOrbit tChief; 
clc; close all
importCast;

%% GLOBALS

global datapath burnTable positionTable chiefOrbit timeTable;
if (isempty(burnTable) || isempty(positionTable) || isempty(chiefOrbit) || isempty(timeTable))
 load burnTable.mat burnTable
 load positionTable.mat positionTable
 load chiefOrbit.mat chiefOrbit
 load timeTable.mat timeTable
end

%% PARAMETER SETUP

datapath = open_logging(true);

measurementPath = "livedata--1-5m--headon--2ndbulb--march8.txt";
data = readmatrix(measurementPath);
data(:, 2) = data(:, 2) / 1000;

writepath = join([datapath, filesep, 'maneuver--test.txt']);
writematrix(["a1_x", "a0_x", "a1_y", "a0_y"], writepath);

%Estimator parameters
estimatorParams = make_estimator_params();
estimatorParams.currentTime = 2*data(1, 1) - data(2, 1);

%Guidance parameters
guidanceParams = make_guidance_params();

collisionEstimate(1) = struct("collisionTime", nan, "predState", nan, "Ppred", nan);

for i = 1 : length(data)
 time = data(i, 1);
 [estimate, estimatorParams] = state_estimator(data(i, 2:3)', time, estimatorParams);
 collisionEstimate = collision_prediction(estimate, estimatorParams, collisionEstimate);
 maneuver = make_maneuver(collisionEstimate, guidanceParams);
 
 if maneuver(3)
  timeToCollision = collisionEstimate.collisionTime - estimatorParams.currentTime;
  [xpoly, ypoly] = make_command(maneuver, timeToCollision, guidanceParams);
  writematrix([xpoly, ypoly], writepath, "WriteMode", "append");
  written = true;
  break;
 end
end
