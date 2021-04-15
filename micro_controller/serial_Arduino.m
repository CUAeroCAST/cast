

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
   metersPerStep = arduinoParams.ratio;
   readnum = (arduinoObj.NumBytesAvailable - mod(arduinoObj.NumBytesAvailable, 2))/2;
   data = read(arduinoObj, readnum, "int16");
   parity = mod(length(data), 2);
   shift = 0;
   if parity
    shift = 1;
   end
   ydata = data(end - shift - 1);
   xdata = data(end - shift);
   data = struct("xpos", xdata*metersPerStep, "ypos",ydata*metersPerStep);
   arduinoObj.UserData = struct("dataReady", true, "feedback", data, "nr", readnum);
  end
 end

 configureCallback(arduinoObj, "byte", readsize, @ arduinoInterrupt);
end
