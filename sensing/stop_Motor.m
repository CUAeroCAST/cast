function [] = stop_Motor(sensorObj)
%STOP_MOTOR Summary of this function goes here
%   Detailed explanation goes here
writestr1 = [165, 37];
writestr2 = [165, 240,2, 0, 0, 87];
if sensorObj.ByteOrder == "little-endian"
 write(sensorObj, writestr1, "uint8")
 pause(1e-2)
 write(sensorObj, writestr2,  "uint8")
else
 write(sensorObj, fliplr(writestr1), "uint8")
 pause(1e-2)
 write(sensorObj, fliplr(writestr2), "uint8")
end

end

