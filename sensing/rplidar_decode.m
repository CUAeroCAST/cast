function packet = rplidar_decode(bits, sensorParams)
packet = struct;
msPerDay = 24 * 3600 * 1000;
if sensorParams.scanMode == "standard"
 bytesPerSample = 5;
 startStart = 0;
 angleStart = 1;
 distanceStart = 3;
 len = sensorParams.readsize / bytesPerSample;
 packet.qual = zeros(1, len);
elseif sensorParams.scanMode == "express"
 len = 0; % NOT IMPLEMENTED YET
end

packet.time = zeros(1, len);
packet.start = zeros(1, len);
packet.angle = zeros(1, len);
packet.distance = zeros(1, len);

sample = 1;
 for i = 1 : length(bits)
  packet.time(sample) = now * (msPerDay); 
  if sensorParams.scanMode == "standard"
   if mod(i, bytesPerSample) == startStart
    packet.start(sample) = mod(bits(i), 3);
    packet.qual(sample) = bits(i) - packet.start(sample);
   elseif mod(i, bytesPerSample) == angleStart
    packet.angle(sample) = typecast(uint8([bits(i), bitshift(bits(i+1), -1)]), "uint16") / 64;
   elseif mod(i, bytesPerSample) == distanceStart
    packet.distance(sample) = typecast(uint8([bits(i), bits(i+1)]), "uint16") / 4;
    sample = sample + 1;
   end
  elseif sensorParams.scanMode == "express"
   packet = bits; % NOT IMPLEMENTED YET
  end
 end
end
