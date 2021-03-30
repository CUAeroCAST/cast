

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
   metersPerStep = .0000705;
   data = read(arduinoObj, arduinoObj.NumBytesAvailable, "uint16");
   parity = mod(length(data), 2);
   shift = 0;
   if parity
    shift = 1;
   end
   xdata = data(end - shift - 1);
   ydata = data(end - shift);
   data = struct("xpos", xdata*metersPerStep, "ypos",ydata*metersPerStep);
   arduinoObj.UserData = struct("dataReady", true, "feedback", data);
  end
 end

 configureCallback(arduinoObj, "byte", readsize, @ arduinoInterrupt);
end
