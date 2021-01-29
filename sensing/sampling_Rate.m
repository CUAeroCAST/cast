function [SR] = sampling_Rate(sensorObj)
%sampling_Rate Summary of this function goes here
%   Detailed explanation goes here

write(sensorObj,[165, 89], "uint8")
SR=read(sensorObj,11,"uint8");

end
