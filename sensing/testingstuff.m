clear
clc
close all

sensorObj = serial_Sensor();
cleanup = onCleanup(@()clean_up(sensorObj));
%%
while true
 if sensorObj.UserData.dataReady
  fprintf(sensorObj.UserData.scan)
  sensorObj.UserData.dataReady = false;
 end
end

function clean_up(sensorObj)
 stop_Motor(sensorObj);
 delete(sensorObj);
end
