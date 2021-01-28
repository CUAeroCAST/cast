% Get health status: a552
% Stop data: a525
% Set motor: a5f0
% Start scan:  a520

% Start Motor: a5f0029402c1
% Stop Motor:  a5f002000057


%serialportlist("all")
sensorObj = serialport("/dev/tty.usbserial-0001",115200);
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
% write(sensorObj,165,"uint8")
% write(sensorObj,82,"uint8")
% read(sensorObj,10,"uint8")

%% Start Motor
start_Motor(sensorObj);


%% Stop Motor
prompt = 'Type "s" to stop motor: ';
stop = input(prompt,'s');
stop_char='s';

if stop == stop_char
    
    stop_Motor(sensorObj);
    
end




delete(sensorObj)



