function params = make_sensor_params()
 params = struct;
 params.readsize = 84;
 params.scanMode = "express";
 [opsys, ~, ~] = compter;
 if contains(opsys, "WIN")
  params.portstr = "COM3";
 else
  params.portstr = "/dev/tty.usbserial-0001";
 end
 params.sensorObj = serial_Sensor(params);
end
