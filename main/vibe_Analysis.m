%% NOTES
% Prior to running test, make sure ports in make_arduino_params and
% make_sensor_params are correct

%% HOUSEKEEPING

clearvars -except burnTable accelTable timeTable positionTable aChief chiefOrbit tChief; 
clc; close all
importCast;

%% GLOBALS

global datapath;

%% CONFIG

log_data = true;

%% PARAMETER SETUP

datapath = open_logging(log_data);

%Arduino parameters
arduinoParams = make_arduino_params();
arduinoCleanup = onCleanup(@()clean_up(arduinoParams.arduinoObj, false));
pause(75);

%Sensor parameters
sensorParams = make_sensor_params();
sensorParams = build_bounding_box(sensorParams);
sensorCleanup = onCleanup(@()clean_up(sensorParams.sensorObj, true));

%Save parameter structs
if log_data
 log_struct(sensorParams, [datapath, filesep, 'sensorParams'])
 log_struct(arduinoParams, [datapath, filesep, 'arduinoParams'])
end

%% VIBRATION

% Constant Velocity Sensor Data
Velocity = 0.1; % m/s
xpoly = [0,Velocity];
ypoly = [0,Velocity];

% Preallocaing Storage Structure
SPD = 24*3600;
measurementStorage(1000) = struct("Obj", nan, "Time", nan, "xs", nan, "ys", nan);

ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'Stop loop', ...
                         'Callback', 'delete(gcbf)');

% Dummy Estimator Params
estimatorParams.xs = 0;
estimatorParams.ys = 0;
[~,~,arduinoParams, sensorParams] = run_io(true, xpoly, ypoly, arduinoParams, estimatorParams, sensorParams);

const=1;
while true
    pause(1e-6)
 % Figure based loop exit, press the button to stop the while loop
 if ~ishandle(ButtonHandle)
  break;
 end

 % filter raw scan for object measurements
 if sensorParams.sensorObj.UserData.dataReady
  measurement.Obj = filter_scan(sensorParams);
  measurement.Time = now * SPD;
  sensorParams.sensorObj.UserData.dataReady = false;
  if measurement.Obj.count > 0
    [measurement.xs, measurement.ys, arduinoParams, sensorParams] = run_io(false, xpoly, ypoly, arduinoParams, estimatorParams, sensorParams);

    measurementStorage(const) = measurement;

    const = const + 1;
  end
 end
 
end

% Save Off Data into the Data Folder
log_struct(measurementStorage,[datapath,filesep,'Vibin'])


