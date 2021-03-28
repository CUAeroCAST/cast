function params = make_sensor_params()
 params = struct;
 params.readsize = 84;
 params.scanMode = "express";
 [opsys, ~, ~] = computer;
 if contains(opsys, "WIN")
  params.portstr = "COM3";
 else
  params.portstr = "/dev/tty.usbserial-0001";
 end
 params.sensorObj = serial_Sensor(params);
 scan_Request(params)

 params.xoffset=0;
 params.yoffset=0;

end
