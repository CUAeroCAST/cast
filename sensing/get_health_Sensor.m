function [Prot_Stop] = get_health_Sensor(sensorObj)
%GET_HEALTH_SENSOR Summary of this function goes here
%   Detailed explanation goes here

write(sensorObj,165,"uint8")
write(sensorObj,82,"uint8")
tic
get_health_output = read(sensorObj,10,"uint8");
time=toc;
healthy = [165, 90,  3, 0, 0, 0, 6, 0, 0, 0];
Prot_Stop=0;

if time>1
    
    error("Connection issue  \n")
    
end

if get_health_output == healthy
    
    fprintf("Sensor health status: good  \n")
    
else
    
    if get_health_output(5)==1
        
            
      fprintf('Get Health Output: [');
      fprintf('%g ', get_health_output);
      fprintf(']\n');
        
        error("Warning: System Detects Potential Risk  %d \n")
        
    end
        
    if get_health_output(5)==2
        
      fprintf('Get Health Output: [');
      fprintf('%g ', get_health_output);
      fprintf(']\n');
      
      Prot_Stop=1;
      
        fprintf("Error: Sensor in Protection Stop State")
        
        
    end
    
    %error("Sensor health  status: bad")
    
end

end

