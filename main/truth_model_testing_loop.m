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
estimatorParams = make_estimator_params(0.01);

%Save parameter structs

n = 100;
N = 50;
alpha = 0.3;
longTimeVec = 0:2.5e-4:1;
nisVec = zeros(1,length(longTimeVec));
neesVec = zeros(1,length(longTimeVec));
numStats = zeros(1,length(longTimeVec));
tMat = [];
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
    fileName = ['consistancyTestRand' fileNum '.mat'];
    load(fileName)
%     fprintf("Setup Complete \n")
    %data(:,3) = deg2rad(data(:,3));
    timeVec = [];
    distance = [];
    angle = [];
    estimatorParams.currentTime = 0;
    xtruth = @(t) 1.5-1.5*t;
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
        if (data(i,3)<180 && data(i+1,3)>180) || i==length(data)-1
            measurement.distance = mean(distance);
            measurement.angle = meanangle(angle);
            time = mean(timeVec);
            [estimate, estimatorParams,nis] = state_estimator([measurement.distance; measurement.angle],...
                                                               time, estimatorParams);
%             collisionEstimate = collision_prediction(estimate, estimatorParams, collisionEstimate);
            estimateStorage(storage) = estimate;
            collisionStorage(storage) = collisionEstimate;
            storage = storage + 1;
            ex = xtruth(time)-estimate.corrState;
            timeIndex = find(abs(longTimeVec-time)<0.0001);
            if numStats(timeIndex)<N
                neesVec(timeIndex) = neesVec(timeIndex)+ ex'*estimate.Pcorr*ex;
                nisVec(timeIndex) = neesVec(timeIndex)+ nis;
                numStats(timeIndex) = numStats(timeIndex)+1;
            end
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
end
neesSum = [];
nisSum = [];
tSum = [];
for k = 1:length(neesVec)
    if numStats(k)==N
        neesSum = [neesSum neesVec(k)];
        nisSum = [nisSum nisVec(k)];
        tSum = [tSum longTimeVec(k)];
    end
end
test_consistancy( neesSum, nisSum, tSum, N, alpha )