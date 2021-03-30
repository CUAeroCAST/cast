function sensorObj = serial_Sensor(sensorParams)

 portstr = sensorParams.portstr;
 readsize = sensorParams.readsize;
 sensorObj = serialport(portstr, 115200, "ByteOrder", "little-endian");
 setDTR(sensorObj, false);
 sensorObj.UserData = struct('dataReady', false, 'scan', nan, 'raw', nan);
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


 
 function sensorInterrupt(sensorObj, info)  
  if ~sensorObj.UserData.dataReady
   availablePkts = floor(sensorObj.NumBytesAvailable / readsize);
   sensorObj.UserData = rplidar_decode(read(sensorObj, readsize*availablePkts, "uint8"), availablePkts, sensorParams, sensorObj);
  end
 end

 configureCallback(sensorObj, "byte", readsize, @ sensorInterrupt);

end
