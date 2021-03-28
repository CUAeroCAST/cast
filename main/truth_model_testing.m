%% HOUSEKEEPING

clearvars -except burnTable accelTable timeTable positionTable aChief chiefOrbit tChief; 
clc; close all
importCast;

%% GLOBALS

global datapath burnTable positionTable chiefOrbit timeTable;
if ~(exist("burnTable", "var") && exist("positionTable", "var") && exist("chiefOrbit", "var") && exist("timeTable", "var"))
 load burnTable.mat burnTable
 load positionTable.mat positionTable
 load chiefOrbit.mat chiefOrbit
 load timeTable.mat timeTable
end

%% CONFIG

log_data = true;

%% PARAMETER SETUP

datapath = open_logging(log_data);

%Estimator parameters
estimatorParams = make_estimator_params();

%Save parameter structs


%% MAIN LOOP

% initialization constants
collisionEstimate(1) = struct("collisionTime", nan, "predState", nan, "Ppred", nan);
moving = false;
SPD = 24*3600;
% ButtonHandle = uicontrol('Style', 'PushButton', ...
%                          'String', 'Stop loop', ...
%                          'Callback', 'delete(gcbf)');
estimateStorage(100) = struct("corrState", nan, "Pcorr", nan, "predState", nan, "Ppred", nan);
collisionStorage(100) = struct("collisionTime", nan, "predState", nan, "Ppred", nan);
storage = 1;

load('C:\Users\Jason\Documents\ASEN4018-4028\git\cast\data\precomputed_sensors\headOn.mat')
fprintf("Setup Complete")
data(:,3) = deg2rad(data(:,3));
timeVec = [];
distance = [];
angle = [];
estimatorParams.currentTime = 0;
ytruth = 0;
vxtruth = -1.5;
vytruth = 0;
x = [];
y = [];
sigx = [];
sigy = [];
vx = [];
vy = [];
sigvx = [];
sigvy = [];
t = [];
for i = 1:length(data)-1
    
    distance = [distance;data(i,2)];
    angle = [angle;data(i,3)];
    timeVec = [timeVec;data(i,1)];
    pause(1e-6) % microsecond pause to enable callback execution

    % filter raw scan for object measurements
    % Estimate the state
    if (data(i,3)<pi && data(i+1,3)>pi) || i==length(data)-1
        measurement.distance = mean(distance);
        measurement.angle = mean(wrapToPi(angle));
        time = mean(timeVec);
        [estimate, estimatorParams] = state_estimator([measurement.distance; measurement.angle],...
                                                           time, estimatorParams);
        collisionEstimate = collision_prediction(estimate, estimatorParams, collisionEstimate);
        estimateStorage(storage) = estimate;
        collisionStorage(storage) = collisionEstimate;
        storage = storage + 1;
        x = [x estimate.corrState(1)];
        y = [y estimate.corrState(2)];
        sigx = [sigx estimate.Pcorr(1,1)];
        sigy = [sigy estimate.Pcorr(2,2)];
        vx = [vx estimate.corrState(3)];
        vy = [vy estimate.corrState(4)];
        sigvx = [vx estimate.Pcorr(3,3)];
        sigvy = [vy estimate.Pcorr(4,4)];
        t = [t time];
        distance = [];
        angle = [];
        timeVec = [];
    end
end
xtruth = 1.5-1.5*t;
xerror = x-xtruth;
yerror = y-ytruth;
vxerror = vx-vxtruth;
vyerror = vy-vytruth;
figure
subplot(2,1,1)
plot(t,x,'k')
hold on
plot(t,x+2*sqrt(sigx),'k--')
plot(t,x-2*sqrt(sigx),'k--')
title('X vs Time')
xlabel('Time(s)')
ylabel('X (m)')
subplot(2,1,2)
plot(t,y,'k')
hold on
plot(t,y+2*sqrt(sigx),'k--')
plot(t,y-2*sqrt(sigx),'k--')
ylim([-.5 .5])
title('Y vs Time')
xlabel('Time(s)')
ylabel('Y (m)')
figure
plot(t,xerror)
%% CLEANUP

%Post Plotting
% plotStruct = make_plotting_params();
% close_logging(plotStruct);
 
% function clean_up(serialObj, motorStop)
% if motorStop
%  stop_Motor(serialObj);
% end
%  delete(serialObj);
% end