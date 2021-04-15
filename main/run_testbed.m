%% HOUSEKEEPING

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

%% CONFIG

log_data = true;

%% PARAMETER SETUP

datapath = open_logging(log_data);

%Arduino parameters
arduinoParams = make_arduino_params();
arduinoCleanup = onCleanup(@()clean_up(arduinoParams.arduinoObj, false));
%Sensor parameters
sensorParams = make_sensor_params();
sensorParams = build_bounding_box(sensorParams);
sensorCleanup = onCleanup(@()clean_up(sensorParams.sensorObj, true));

%Estimator parameters
estimatorParams = make_estimator_params();

%Guidance parameters
guidanceParams = make_guidance_params();

%Save parameter structs
if log_data
 log_struct(estimatorParams, [datapath, filesep, 'estimatorParams'])
 log_struct(sensorParams, [datapath, filesep, 'sensorParams'])
 log_struct(guidanceParams, [datapath, filesep, 'guidanceParams'])
 log_struct(arduinoParams, [datapath, filesep, 'arduinoParams'])
end

%% MAIN LOOP

% initialization constants
collisionEstimate(1) = struct("collisionTime", nan, "predState", nan, "Ppred", nan);
moving = false;
SPD = 24*3600;
ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'Stop loop', ...
                         'Callback', 'delete(gcbf)');
maxStorage = 150;
estimateStorage(maxStorage) = struct("corrState", nan, "Pcorr", nan, "predState", nan, "Ppred", nan);
collisionStorage(maxStorage) = struct("collisionTime", nan, "predState", nan, "Ppred", nan);
measurementStorage(maxStorage) = struct("time", nan, "distance", nan, "angle", nan, "count", nan, "xs", nan, "ys", nan);
storage = 1;

firstReading = true;
fprintf("Setup Complete\n")
while true
 pause(1e-6) % microsecond pause to enable callback execution
 % Figure based loop exit, press the button to stop the while loop
 if ~ishandle(ButtonHandle)
  break;
 end

 % filter raw scan for object measurements
 if sensorParams.sensorObj.UserData.dataReady
  measurement = filter_scan(sensorParams);
  time = now * SPD;
  measurement.time = time;
  sensorParams.sensorObj.UserData.dataReady = false;
  
  if measurement.count > 0
    if firstReading
     estimatorParams.currentTime = time - 0.1;
     firstReadingTime = time - 0.1;
     firstReading = false;
    end
    % Estimate the state
    [estimate, estimatorParams] = state_estimator([measurement.distance; measurement.angle],...
                                                       time, estimatorParams);
    collisionEstimate = collision_prediction(estimate, estimatorParams, collisionEstimate);

    % calculate maneuver
    if ~moving
     maneuver = make_maneuver(collisionEstimate, guidanceParams);
     if maneuver(3)
      arduinoParams = set_stops(collisionEstimate, arduinoParams, guidanceParams);
      timeToCollision = collisionEstimate.collisionTime - estimatorParams.currentTime;
      [xpoly, ypoly] = make_command(maneuver, timeToCollision, guidanceParams);
      moving = 1;
      moveTime = time;
      [estimatorParams.xs, estimatorParams.ys, arduinoParams, sensorParams] = run_io(true, xpoly, ypoly, arduinoParams, estimatorParams, sensorParams);
      maneuver = [maneuver, moveTime];
     end
    end
    [estimatorParams.xs, estimatorParams.ys, arduinoParams, sensorParams] = run_io(false, 0, 0, arduinoParams, estimatorParams, sensorParams);
    measurement.xs = estimatorParams.xs;
    measurement.ys = estimatorParams.ys;
    estimateStorage(storage) = estimate;
    collisionStorage(storage) = collisionEstimate;
    measurementStorage(storage) = measurement;
    storage = storage + 1;
  end
 end
end
%% CLEANUP

%Post Plotting
% plotStruct = make_plotting_params();
% close_logging(plotStruct);

log_struct(estimateStorage, [datapath, filesep, 'estimateStorage']);
log_struct(measurementStorage, [datapath, filesep, 'measurementStorage']);
log_struct(collisionStorage, [datapath, filesep, 'colisionStorage']);
log_struct(maneuver, [datapath, filesep, 'maneuver']);

clean_up(sensorParams.sensorObj, true);
clean_up(arduinoParams.arduinoObj, false);
