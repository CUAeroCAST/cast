function params = make_arduino_params()
 [opsys, ~, ~] = computer;
 if contains(opsys, "WIN")
  params.portstr = "COM5";
 else
  params.portstr = "/dev/tty.usbserial-0001";
 end
 params.readsize = 4;
 params.ratio = 0.0000705*1.4;
 params.arduinoObj = serial_Arduino(params);
 pause(20)
 flush(params.arduinoObj);
 params.xStop = 0.4;
 params.yStop = 0.4;
end
