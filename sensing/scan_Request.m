function [] = scan_Request(sensorObj)
%SCAN_REQUEST Summary of this function goes here
%   Detailed explanation goes here
writestr = [165, 32];
if sensorObj.ByteOrder == "litter-endian"
 write(sensorObj, writestr,  "uint8")
else
 write(sensorObj, fliplr(writestr), "uint8")
end
%Data=read(sensorObj,16,"uint8");

end

