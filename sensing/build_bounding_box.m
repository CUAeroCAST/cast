function sensorParams = build_bounding_box(sensorParams)
samplesPerScan = 400;
first_scan = struct;
first_scan.distance = [];
first_scan.angle = [];

while length(first_scan.distance) < samplesPerScan
 pause(1e-6)
 if sensorParams.sensorObj.UserData.dataReady
  for i = 1 : length(sensorParams.sensorObj.UserData.scan)
   first_scan.distance = [first_scan.distance; sensorParams.sensorObj.UserData.scan(i).distance'];
   first_scan.angle = [first_scan.angle; sensorParams.sensorObj.UserData.scan(i).angle'];
  end
  sensorParams.sensorObj.UserData.dataReady = false;
 end
end

[sensorParams.boundingbox.cornerx, sensorParams.boundingbox.cornery]... 
 = sensor_Bounds(first_scan);
end
