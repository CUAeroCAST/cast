function [] = start_Motor(sensorObj)
%START_MOTOR Summary of this function goes here
%   Detailed explanation goes here

% Start Motor: a5f0029402c1

write(sensorObj,[165, 240, 2, 148, 2, 193], "uint8") % CURRENTLY 5HZ

end

