function [SR] = sampling_Rate(sensorObj)
%sampling_Rate Summary of this function goes here
%   Detailed explanation goes here
writestr = [165, 89];
if sensorObj.ByteOrder == "little-endian"
 write(sensorObj, writestr,  "uint8")
else
 write(sensorObj, fliplr(writestr), "uint8")
end
SR=read(sensorObj,11,"uint8");

end
