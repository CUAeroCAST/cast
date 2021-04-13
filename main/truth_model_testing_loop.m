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
estimatorParams = make_estimator_params(0.001);

%Save parameter structs

n = 50;
N = 50;
alpha = 0.3;
longTimeVec = 0:2.5e-4:1;
nisVec = zeros(1,length(longTimeVec));
neesVec = zeros(1,length(longTimeVec));
numStats = zeros(1,length(longTimeVec));
nisVec = [];
neesVec = [];
storeTimes = [];
tMat = [];
stateError = [];
innovation = [];
measErrorVec = [];
x = [];
y = [];
sigx = [];
sigy = [];
vx = [];
vy = [];
sigvx = [];
sigvy = [];
t = [];
xErrorVec = [];
%% MAIN LOOP
for j = 1:n
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
    
    fileNum = num2str(j);
    fileName = ['consistancyTestRandState' fileNum '.mat'];
    load(fileName)
    data = dataStruct.data;
    state = dataStruct.state;
%     fprintf("Setup Complete \n")
    %data(:,3) = deg2rad(data(:,3));
    timeVec = [];
    distance = [];
    angle = [];
    truthState = [];
    estimatorParams.currentTime = 0;
    xtruth = @(t) [1.5-1.5*t;0;-1.5;0];
    x = [];
    y = [];
    sigx = [];
    sigy = [];
    vx = [];
    vy = [];
    sigvx = [];
    sigvy = [];
    errorVec = [];
    t = [];
    estimatorParams.filter.State = [cosd(data(1,3))*data(1,2); 0; -1.5; 0];
    for i = 2:length(data)
        distance = [distance;data(i,2)];
        angle = [angle;data(i,3)];
        timeVec = [timeVec;data(i,1)];
        truthStateIndex = find(data(i,1)==state(5,:));
        truthState = [truthState state(1:4,truthStateIndex)];
        measTruth = sqrt(state(1,truthStateIndex)^2+state(2,truthStateIndex)^2);
        measError = measTruth-data(i,2);
        pause(1e-6) % microsecond pause to enable callback execution

        % filter raw scan for object measurements
        % Estimate the state
        if i==length(data)
            measurement.distance = mean(distance);
            measurement.angle = deg2rad(meanangle(angle));
            time = mean(timeVec);
            if length(angle)>1
                truthState = mean(truthState')';
            end
            [estimate, estimatorParams,nis,diffs] = state_estimator([measurement.distance; measurement.angle],...
                                                               time, estimatorParams);
            innovation = [innovation diffs];
%             collisionEstimate = collision_prediction(estimate, estimatorParams, collisionEstimate);
            estimateStorage(storage) = estimate;
            collisionStorage(storage) = collisionEstimate;
            storage = storage + 1;
            ex = truthState-estimate.corrState;
            stateError = [stateError ex];
            timeIndex = find(abs(longTimeVec-time)<0.0001);
%             if numStats(timeIndex)<N
%                 neesVec(timeIndex) = neesVec(timeIndex)+ ex'*inv(estimate.Pcorr)*ex;
%                 nisVec(timeIndex) = nisVec(timeIndex)+ nis;
%                 numStats(timeIndex) = numStats(timeIndex)+1;
%             end
            neesStat = ex'*inv(estimate.Pcorr)*ex;
            neesVec = [neesVec neesStat];
            nisVec = [nisVec nis];
            storeTimes = [storeTimes time];
            x = [x estimate.corrState(1)];
            y = [y estimate.corrState(2)];
            sigx = [sigx estimate.Pcorr(1,1)];
            sigy = [sigy estimate.Pcorr(2,2)];
            vx = [vx estimate.corrState(3)];
            vy = [vy estimate.corrState(4)];
            sigvx = [sigvx estimate.Pcorr(3,3)];
            sigvy = [sigvy estimate.Pcorr(4,4)];
            errorVec = [errorVec ex];
            xErrorVec = [xErrorVec ex(1,:)];
            t = [t time];
            distance = [];
            angle = [];
            timeVec = [];
            truthState = [];
        elseif (data(i,3)<180 && data(i+1,3)>180)
            measurement.distance = mean(distance);
            measurement.angle = deg2rad(meanangle(angle));
            time = mean(timeVec);
            if length(angle)>1
                truthState = mean(truthState')';
            end
            [estimate, estimatorParams,nis,diffs] = state_estimator([measurement.distance; measurement.angle],...
                time, estimatorParams);
            innovation = [innovation diffs];
%             collisionEstimate = collision_prediction(estimate, estimatorParams, collisionEstimate);
            estimateStorage(storage) = estimate;
            collisionStorage(storage) = collisionEstimate;
            storage = storage + 1;
            ex = truthState-estimate.corrState;
            stateError = [stateError ex];
            timeIndex = find(abs(longTimeVec-time)<0.0001);
%             if numStats(timeIndex)<N
%                 neesVec(timeIndex) = neesVec(timeIndex)+ ex'*inv(estimate.Pcorr)*ex;
%                 nisVec(timeIndex) = nisVec(timeIndex)+ nis;
%                 numStats(timeIndex) = numStats(timeIndex)+1;
%             end
            neesStat = ex'*inv(estimate.Pcorr)*ex;
            neesVec = [neesVec neesStat];
            nisVec = [nisVec nis];
            storeTimes = [storeTimes time];
            x = [x estimate.corrState(1)];
            y = [y estimate.corrState(2)];
            sigx = [sigx estimate.Pcorr(1,1)];
            sigy = [sigy estimate.Pcorr(2,2)];
            vx = [vx estimate.corrState(3)];
            vy = [vy estimate.corrState(4)];
            sigvx = [sigvx estimate.Pcorr(3,3)];
            sigvy = [sigvy estimate.Pcorr(4,4)];
            errorVec = [errorVec ex];
            xErrorVec = [xErrorVec ex(1,:)];
            t = [t time];
            distance = [];
            angle = [];
            timeVec = [];
            truthState = [];
        end
    end
    
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
figure(1)
subplot(2,2,1)
plot(t,errorVec(1,:),'k-')
hold on
plot(t,2*sqrt(sigx),'k--');
plot(t,-2*sqrt(sigx),'k--');
ylim([-.05 .05])
subplot(2,2,2)
plot(t,errorVec(2,:),'k-')
hold on
plot(t,2*sqrt(sigy),'k--');
plot(t,-2*sqrt(sigy),'k--');
subplot(2,2,3)
plot(t,errorVec(3,:),'k-')
hold on
plot(t,2*sqrt(sigvx),'k--');
plot(t,-2*sqrt(sigvx),'k--');
subplot(2,2,4)
plot(t,errorVec(4,:),'k-')
hold on
plot(t,2*sqrt(sigvy),'k--');
plot(t,-2*sqrt(sigvy),'k--');
clf
end
% neesSum = [];
% nisSum = [];
% tSum = [];
% for k = 1:length(neesVec)
%     if numStats(k)==N
%         neesSum = [neesSum neesVec(k)];
%         nisSum = [nisSum nisVec(k)];
%         tSum = [tSum longTimeVec(k)];
%     end
% end
% neesSum = neesSum./N;
% nisSum = nisSum./N;


figure 
subplot(1,2,1)
scatter(storeTimes,innovation(1,:))
title('Range Innovation')
subplot(1,2,2)
scatter(storeTimes,innovation(2,:))
title('Bearing Innovation')
test_consistancy( neesVec, nisVec, storeTimes, N, alpha )