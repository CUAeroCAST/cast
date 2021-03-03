function response = scan_Request(sensorParams)
%SCAN_REQUEST Summary of this function goes here
%   Detailed explanation goes here
 global datapath;
 sensorObj = sensorParams.sensorObj;
 if sensorParams.scanMode == "standard"
  writestr = [0xA5, 0x20];
  respstr = [0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81];
 elseif sensorParams.scanMode == "express"
  writestr = [0xA5, 0x82, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00];
  writestr = [writestr, rplider_checksum(writestr)];
  respstr = [0xA5, 0x5A, 0x84, 0x00, 0x00, 0x40, 0x84];
 end

 if sensorObj.ByteOrder == "little-endian"
  write(sensorObj, writestr,  "uint8")
 else
  write(sensorObj, fliplr(writestr), "uint8")
 end

 response = read(sensorObj, 7, "uint8");
 if ~all(response == respstr)
  event = MException('sensor:scan_requst', 'Scan Request Failed.');
  log_event(datapath, event);
  throw(event)
 end
end

