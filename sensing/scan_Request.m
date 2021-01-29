function [] = scan_Request(sensorObj)
%SCAN_REQUEST Summary of this function goes here
%   Detailed explanation goes here

write(sensorObj,[165, 32], "uint8")
%Data=read(sensorObj,16,"uint8");

end

