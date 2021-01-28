
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
function serial_Sensor
%serialportlist("all")
sensorObj = serialport("/dev/tty.usbserial-0001",115200);
cleanup = onCleanup(@()stop_Motor(sensorObj));
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

%% Scan Request
start_Motor(sensorObj);

data = struct;
data.scan = zeros(1,16);
data.readyFlag = false;
data.running = true;
set(0, 'userdata', data);

%while(data.running)
for i=1:1000
  data = get(0, 'userdata');
  if ~data.readyFlag
   data.scan = read(sensorObj, 16, 'uint8');
   data.readyFlag = true;
  end
  set(0, 'userdata', data);
end

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


