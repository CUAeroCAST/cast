function [] = sensor_Reset(sensorObj)
%STOP_MOTOR Summary of this function goes here
%   Detailed explanation goes here
writestr = [165, 64];
if sensorObj.ByteOrder == "little-endian"
 write(sensorObj, writestr,  "uint8")
else
 write(sensorObj, fliplr(writestr), "uint8")
end

end
