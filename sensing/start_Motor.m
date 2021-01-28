function [] = start_Motor(sensorObj)
%START_MOTOR Summary of this function goes here
%   Detailed explanation goes here

% Start Motor: a5f0029402c1

% write(sensorObj,165,"uint8")
% write(sensorObj,240,"uint8")
% write(sensorObj,2,"uint8")
% write(sensorObj,148,"uint8")
% write(sensorObj,2,"uint8")
% write(sensorObj,193,"uint8")

write(sensorObj,[165, 240, 2, 148, 2, 193], "uint8")
%Sample = read(sensorObj,10,"uint8");
%write(sensorObj,[165, 32], "uint8")

end

