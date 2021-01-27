% Get health status: a552
% Stop data: a525
% Set  motor: a5f0
% Start  scan:  a520

% Start Motor: a5f0029402c1
% Stop Motor:  a5f002000057


%serialportlist("all")
sensorObj = serialport("/dev/tty.usbserial-0001",115200);
flush(sensorObj)

%% Get health status
get_health_Sensor(sensorObj)
% write(sensorObj,165,"uint8")
% write(sensorObj,82,"uint8")
% read(sensorObj,10,"uint8")

%% Start Motor
start_Motor(sensorObj)


%% Stop Motor
prompt = 'Type "s" to stop motor: ';
stop = input(prompt,'s');
stop_char='s';

if stop == stop_char
    
    stop_Motor(sensorObj);
    
end




delete(sensorObj)



