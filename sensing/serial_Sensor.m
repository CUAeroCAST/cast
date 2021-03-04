
%clc
%clear
%close all

% Get health status: a552
% Stop data: a525
% Set motor: a5f0
% Start scan:  a520

% Start Motor: a5f0029402c1
% Stop Motor:  a5f002000057

function sensorObj = serial_Sensor(sensorParams)

 portstr = sensorParams.portstr;
 readsize = sensorParams.readsize;
 [~, ~, endian] = computer;
  if endian == 'L'
   sensorObj = serialport(portstr,115200, "ByteOrder", "little-endian");
  else
   sensorObj = serialport(portstr, 115200, "ByteOrder", "big-endian");
  end
 sensorObj.UserData = struct('dataReady', false, 'scan', zeros(1, readsize), 'totalRead', 0);
 flush(sensorObj)

%% Get health status

 count=0;
 [Prot_Stop]=get_health_Sensor(sensorObj);

 if Prot_Stop==1
  while Prot_Stop==1 && count<=3
      
   count=count+1;
   sensor_Reset(sensorObj)
   pause(0.002)
   
   [Prot_Stop]=get_health_Sensor(sensorObj);
   
  end
 end

 if Prot_Stop==1 && count>3
    
  error("Error: Sensor in Protection Stop State and is not responding to RESET")
    
 end


 
 function sensorInterrupt(sensorObj,info)
  
  if ~sensorObj.UserData.dataReady
   sensorObj.UserData = struct('dataReady', true,...
                                                 'scan', rplidar_decode(read(sensorObj, readsize, "uint8"), sensorParams),...
                                                 'totalRead', sensorObj.UserData.totalRead);
  end
   sensorObj.UserData.totalRead = sensorObj.UserData.totalRead + readsize;
 end

 configureCallback(sensorObj, "byte", readsize, @ sensorInterrupt);

end
