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
 log_struct(estimatorParams, [datapath, filesep, "estimatorParams"])
 log_struct(sensorParams, [datapath, filesep, "sensorParams"])
 log_struct(guidanceParams, [datapath, filesep, "guidanceParams"])
 log_struct(arduinoParams, [datapath, filesep, "arduinoParams"])
end

%% MAIN LOOP

% initialization constants
collisionEstimate(1) = struct("collisionTime", nan, "predState", nan, "Ppred", nan);
moving = false;
SPD = 24*3600;
ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'Stop loop', ...
                         'Callback', 'delete(gcbf)');
estimateStorage(100) = struct("corrState", nan, "Pcorr", nan, "predState", nan, "Ppred", nan);
collisionStorage(100) = struct("collisionTime", nan, "predState", nan, "Ppred", nan);
storage = 1;

fprintf("Setup Complete")
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
  sensorParams.sensorObj.UserData.dataReady = false;
  
  if measurement.count > 0

    % Estimate the state
    [estimate, estimatorParams] = state_estimator([measurement.distance; measurement.angle],...
                                                       time, estimatorParams);
    collisionEstimate = collision_prediction(estimate, estimatorParams, collisionEstimate);
    estimateStorage(storage) = estimate;
    collisionStorage(storage) = collisionEstimate;
    storage = storage + 1;

    % calculate maneuver
    if ~moving
     maneuver = make_maneuver(collisionEstimate, guidanceParams);
     if maneuver(3)
      [xpoly, ypoly] = make_command(maneuver, collisionEstimate.collisionTime - estimatorParams.currentTime, guidanceParams);
      moving = 1;
     end
    end
    
    [estimatorParams.xs, estimatorParams.ys, arduinoParams] = run_io(true, xpoly, ypoly, arduinoParams, estimatorParams);
  end
 end
end
%% CLEANUP

%Post Plotting
% plotStruct = make_plotting_params();
% close_logging(plotStruct);
 
function clean_up(serialObj, motorStop)
if motorStop
 stop_Motor(serialObj);
end
 delete(serialObj);
end
