
%clc
%clear
%close all

% Get health status: a552
% Stop data: a525
% Set motor: a5f0
% Start scan:  a520

% Start Motor: a5f0029402c1
% Stop Motor:  a5f002000057
%run_sensor();
function sensorObj = serial_Sensor
%serialportlist("all")
portstr = "/dev/tty.usbserial-0001";
readsize = 2000;
[str, maxsize, endian] = computer;
 if endian == 'L'
sensorObj = serialport(portstr,115200);
 else
  sensorObj = serialport(portstr, 115200, "ByteOrder", "big-endian");
 end
sensorObj.UserData = struct('dataReady', false, 'scan', zeros(1,5), 'totalRead', 0);
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
% write(sensorObj,165,"uint8")
% write(sensorObj,82,"uint8")
% read(sensorObj,10,"uint8")

 function sensorInterrupt(sensorObj,info)
     %disp(sensorObj)
  totalRead = sensorObj.UserData.totalRead;
  if sensorObj.UserData.totalRead >= 7
      readsize_actual = readsize;
  else
      readsize_actual = 7;
  end
  if sensorObj.UserData.dataReady && sensorObj.UserData.totalRead >=7
   %sensorObj.UserData.scan = [sensorObj.UserData.scan, read(sensorObj, readsize_actual, "uint8")'];
  else
   sensorObj.UserData = struct('dataReady', true, 'scan', read(sensorObj, readsize_actual, "uint8")', 'totalRead', totalRead);
  end
  sensorObj.UserData.totalRead = sensorObj.UserData.totalRead + readsize_actual;
 end

configureCallback(sensorObj,"byte",readsize,@ sensorInterrupt);
%  sensorObj.BytesAvailableFcnMode = "byte";
%  sensorObj.BytesAvailableFcnCount = readsize;
%  sensorObj.BytesAvailableFcn = @()sensorInterrupt;

end
%% Start Motor

% start_Motor(sensorObj);
% [Data] = scan_Request(sensorObj);
% 
% onCleanup(
% matlab -batch serial_Sensor
% 
% while true
% x=read(sensorObj,16,"uint8");
% 
% count2=2;
% %% Stop Motor
% prompt = 'Type 1 to stop motor: ';
% %stop = input(prompt,'s');
% %stop_char='s';
% stopcheck=0;
% try
% while true
% %stop = input(prompt);
% 
% 	%if stopcheck==0
%         [Data(count2,:)] = scan_Request(sensorObj);
%         count2=count2+1;
% end
%         
%     %end
% catch 
% 
%     %if stop == 1
%         %stopcheck=1;
%         stop_Motor(sensorObj);
%         %break 
% end
% 
%          
% 
% 
% 
% % if stop == stop_char
% %     
% %     stop_Motor(sensorObj);
% %    
% % end
% 
% 
% 
% delete(sensorObj)


