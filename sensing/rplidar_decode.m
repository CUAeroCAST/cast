function UserData = rplidar_decode(bits, numPkts, sensorParams, sensorObj)
 readsize = sensorParams.readsize;
 UserData = struct("dataReady", true, "scan", nan, "raw", nan);
 if sensorParams.scanMode == "standard"
  UserData.scan = standard_decode(bits);
 elseif sensorParams.scanMode == "express"
  % express scan requires 2 packets to realize the values of the angles
  if isstruct(sensorObj.UserData.raw)
   % first packet already exists, merge packets for values
   [UserData.scan, UserData.raw] = express_merge(sensorObj.UserData.raw, bits, numPkts, sensorParams);
  else
   if numPkts > 1
    % get first packet, run merge on remainder
    raw = express_decode(bits(1 : sensorParams.readsize));
    [UserData.scan, UserData.raw] = express_merge(raw, bits(readsize + 1 : 2*readsize), numPkts-1, sensorParams);
   else
    % only initialization packet is available.
    UserData.raw = express_decode(bits);
    UserData.dataReady = false;
   end
  end
 end
end
