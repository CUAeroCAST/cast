function UserData = rplidar_decode(bits, sensorParams, sensorObj)
 UserData = struct("dataReady", true);
 if sensorParams.scanMode == "standard"
  UserData.scan = standard_decode(bits);
 elseif sensorParams.scanMode == "express"
  if isstruct(sensorObj.raw)
   [UserData.scan, UserData.raw] = express_merge(raw, bits);
  else
   UserData.raw = express_decode(bits);
  end
 end
end
