

function arduinoObj = serial_Arduino(arduinoParams)

 portstr = arduinoParams.portstr;
 readsize = arduinoParams.readsize;
 [~, ~, endian] = computer;
%   if endian == 'L'
%    arduinoObj = serialport(portstr,115200, "ByteOrder", "little-endian");
%   else
%    arduinoObj = serialport(portstr, 115200, "ByteOrder", "big-endian");
%   end
 arduinoObj = serialport(portstr, 115200);
 arduinoObj.UserData = struct('dataReady', false, 'feedback', nan);
 flush(arduinoObj)

 %% INTERRUPT
 
 function arduinoInterrupt(arduinoObj, info)  
  if ~arduinoObj.UserData.dataReady
   xdata = read(arduinoObj, 1, "int16");
   ydata = read(arduinoObj, 1, "int16");
   data = struct("xpos", xdata*0.0000705, "ypos",ydata*0.0000705);
   arduinoObj.UserData = struct("dataReady", true, "feedback", data);
%    arduinoObj.UserData = rplidar_decode(read(arduinoObj, readsize, "single"), sensorParams, arduinoObj);
  end
 end

 configureCallback(arduinoObj, "byte", readsize, @ arduinoInterrupt);
end
