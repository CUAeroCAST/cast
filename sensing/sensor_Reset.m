function [] = sensor_Reset(sensorObj)
%STOP_MOTOR Summary of this function goes here
%   Detailed explanation goes here

write(sensorObj,[165, 64], "uint8")

end