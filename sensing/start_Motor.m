function [] = start_Motor(sensorObj)
%START_MOTOR Summary of this function goes here
%   Detailed explanation goes here

% Start Motor: a5f0029402c1
writestr = [0xA5; 0xF0; 0x02; 0xFF; 0x03];
writestr = [writestr; rplidar_checksum(writestr)];

if sensorObj.ByteOrder == "little-endian"
 for i = 1:length(writestr)
 write(sensorObj, writestr(i),  "uint8") % CURRENTLY 5HZ
 pause(1e-6)
 end
else
 write(sensorObj, fliplr(writestr), "uint8")
 pause(1e-3)
end

end

