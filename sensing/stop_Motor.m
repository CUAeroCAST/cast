function [] = stop_Motor(sensorObj)
%STOP_MOTOR Summary of this function goes here
%   Detailed explanation goes here

% write(sensorObj,165,"uint8")
% write(sensorObj,240,"uint8")
% write(sensorObj,2,"uint8")
% write(sensorObj,0,"uint8")
% write(sensorObj,0,"uint8")
% write(sensorObj,87,"uint8")

write(sensorObj,[165, 240, 2, 0, 0, 87], "uint8")

end

