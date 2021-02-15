function [] = start_Motor(sensorObj)
%START_MOTOR Summary of this function goes here
%   Detailed explanation goes here

% Start Motor: a5f0020294c1
writestr = [165, 240, 2, 2, 148, 193];
if sensorObj.ByteOrder == "little-endian"
 write(sensorObj, writestr,  "uint8") % CURRENTLY 5HZ
else
 write(sensorObj, fliplr(writestr), "uin8")
end

end

