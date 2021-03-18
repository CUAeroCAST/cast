function params = make_arduino_params()
 [opsys, ~, ~] = compter;
 if contains(opsys, "WIN")
  params.portstr = "COM3";
 else
  params.portstr = "/dev/tty.usbserial-0001";
 end
 params.readsize = 4;
 params.arduinoObj = serialArduino(params);
 params.xStop = 0.4;
 params.yStop = 0.4;
end
