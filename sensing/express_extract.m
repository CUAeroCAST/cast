clear
clc
close all

blah()
function blah()
filename = "data.txt";
writematrix(["data"], filename);

sensorParams = struct;
sensorParams.readsize = 128;
sensorParams.scanMode = "express";
sensorParams.portstr = "COM3";

sensorParams.sensorObj = serial_Sensor(sensorParams);
cleanup = onCleanup(@()clean_up(sensorParams.sensorObj));
%%
% start_Motor(sensorObj);
SR = sampling_Rate(sensorParams.sensorObj);
scan_Request(sensorParams);
while true
 if sensorParams.sensorObj.UserData.dataReady
  writematrix(sensorParams.sensorObj.UserData.scan, filename, 'WriteMode', 'append');
  sensorParams.sensorObj.UserData.dataReady = false;
 end
end
end

function clean_up(sensorObj)
 stop_Motor(sensorObj);
 delete(sensorObj);
end
