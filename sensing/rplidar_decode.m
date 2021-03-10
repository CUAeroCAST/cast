function UserData = rplidar_decode(bits, sensorParams, sensorObj)
 UserData = struct("dataReady", true, "scan", nan, "raw", nan);
 if sensorParams.scanMode == "standard"
  UserData.scan = standard_decode(bits);
 elseif sensorParams.scanMode == "express"
  if isstruct(sensorObj.UserData.raw)
   [UserData.scan, UserData.raw] = express_merge(sensorObj.UserData.raw, bits);
  else
   UserData.raw = express_decode(bits);
   UserData.dataReady = false;
  end
 end
end
