clearvars -except burnTable accelTable timeTable positionTable aChief chiefOrbit tChief; 
clc; close all
importCast;

%% GLOBALS

global datapath burnTable positionTable chiefOrbit timeTable;
if (isempty(burnTable) || isempty(positionTable) || isempty(chiefOrbit) || isempty(timeTable))
 load burnTable.mat burnTable
 load positionTable.mat positionTable
%  load chiefOrbit.mat chiefOrbit
%  load timeTable.mat timeTable
end

%% PARAMETER SETUP

datapath = open_logging(true);

measurementPath = "sensor_data.txt";
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
 fprintf("Current time = %.2f\n", time - data(1,1))
 figure
 xlim([-0.1,2])
 ylim([-0.6,0.6])
 hold on
 plot(0,0,'go')
 plot(estimate.predState(1), estimate.predState(2), 'ro')
 ellipse(estimate.predState(1), estimate.predState(2), 2*sqrt(estimate.Ppred(1,1)), 2*sqrt(estimate.Ppred(2,2)), '--k');
 plot(estimate.corrState(1), estimate.corrState(2), 'bo')
 ellipse(estimate.corrState(1), estimate.corrState(2), 2*sqrt(estimate.Pcorr(1,1)), 2*sqrt(estimate.Pcorr(2,2)), '--k');
 plot(collisionEstimate.predState(1), collisionEstimate.predState(2), 'mo')
 ellipse(collisionEstimate.predState(1), collisionEstimate.predState(2), 2*sqrt(collisionEstimate.Ppred(1,1)), 2*sqrt(collisionEstimate.Ppred(2,2)), '--k');
 if maneuver(3)
     fprintf("Command Made\n")
     plot([0,maneuver(3)*maneuver(1)], [0, maneuver(3)*maneuver(2)], 'r')
     break;
 end
 close
end
