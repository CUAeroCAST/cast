function [] = get_health_Sensor(sensorObj)
%GET_HEALTH_SENSOR Summary of this function goes here
%   Detailed explanation goes here

write(sensorObj,165,"uint8")
write(sensorObj,82,"uint8")
get_health_output = read(sensorObj,10,"uint8");
healthy = [165, 90,  3, 0, 0, 0, 6, 0, 0, 0];

if get_health_output == healthy
    
    fprintf("Sensor health status: good  \n")
    
else 
    
    error("Sensor health  status: bad")
    
end

end

