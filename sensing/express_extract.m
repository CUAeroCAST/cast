clear
clc
close all

blah()
function blah()
filename = "expressdata--empty4.txt";
writematrix(["distance","angle"], filename);

sensorParams = struct;
sensorParams.readsize = 84;
sensorParams.scanMode = "express";
sensorParams.portstr = "/dev/tty.usbserial-0001";

sensorParams.sensorObj = serial_Sensor(sensorParams);
cleanup = onCleanup(@()clean_up(sensorParams.sensorObj));
%%
% start_Motor(sensorObj);
SR = sampling_Rate(sensorParams.sensorObj);
scan_Request(sensorParams);
while true
 pause(1e-6)
 if sensorParams.sensorObj.UserData.dataReady && isstruct(sensorParams.sensorObj.UserData.scan)
  writematrix([sensorParams.sensorObj.UserData.scan.distance', sensorParams.sensorObj.UserData.scan.angle'], filename, 'WriteMode', 'append');
  sensorParams.sensorObj.UserData.dataReady = false;
 end
end
end

function clean_up(sensorObj)
 stop_Motor(sensorObj);
 delete(sensorObj);
end
