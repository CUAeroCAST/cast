function [] = stop_Motor(sensorObj)
%STOP_MOTOR Summary of this function goes here
%   Detailed explanation goes here

write(sensorObj,[165, 37],"uint8")
pause(1e-2)
write(sensorObj,[165, 240, 2, 0, 0, 87], "uint8")

end

