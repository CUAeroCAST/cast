clear
clc
close all
importCast;
SPD = 24 * 3600;

filename = "state_estimator_live.txt";
writematrix(["time", "xpos", "ypos", "xcol", "ycol", "xxcov", "xycov", "yycov"], filename);

sensorParams = struct;
sensorParams.scanMode = "express";
sensorParams.readsize = 84;
sensorParams.portstr = "/dev/tty.usbserial-0001";
sensorParams.sensorObj = serial_Sensor(sensorParams);

estimatorParams = struct;
estimatorParams.llsSeeding = false;
estimatorParams.batchSamples = 0;
estimatorParams.sensorCovariance = [0.025^2,0;0,deg2rad(0.45)^2];
estimatorParams.qGain = 1;
estimatorParams.initState = [1.5;-1;0;0];
[~, estimatorParams] = init_estimator([nan, nan], estimatorParams);
estimatorParams.time = now*SPD;

first_scan = struct;
first_scan.distance = [];
first_scan.angle = [];

while length(first_scan.distance) < 800
 pause(1e-6)
 if sensorParams.sensorObj.UserData.dataReady
  first_scan.distance = [first_scan.distance, sensorParams.sensorObj.UserData.scan.distance];
  first_scan.angle = [first_scan.angle, sensorParams.sensorObj.UserData.scan.angle];
  sensorParams.sensorObj.UserData.dataReady = false;
 end
end

[sensorParams.boundingbox.r12, sensorParams.boundingbox.r13,...
 sensorParams.boundingbox.r24, sensorParams.boundingbox.r43]... 
 = sensor_Bounds(first_scan);

while true
 pause(1e-6)
 if sensorParams.sensorObj.UserData.dataReady
  measurement = filter_scan(sensorParams);
  sensorParams.sensorObj.UserData.dataReady = false;
  
  if measurement.count > 0
   
  
  