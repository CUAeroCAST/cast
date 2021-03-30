function [packets, packet2] = express_merge(packet1, bits, numPkts, sensorParams)
 readsize = sensorParams.readsize;
 if numPkts > 1
  packets(numPkts - 1) = packet1;
  for i = 0 : numPkts - 1
  packet2 = express_decode(bits(readsize*i + 1: readsize*(i+1)));
  alpha = packet2.startAngle - packet1.startAngle;
  if alpha < 0
   alpha = alpha + 360;
  end
  len = length(packet1.distance);
  packet1.angle = packet1.startAngle + alpha/32 * (0:len-1) - packet1.angle;
  packets(i+1) = packet1;
  packet1 = packet2;
 end
 else
  packet2 = express_decode(bits);
  alpha = packet2.startAngle - packet1.startAngle;
  if alpha < 0
   alpha = alpha + 360;
  end
  len = length(packet1.distance);
  packet1.angle = packet1.startAngle + alpha/32 * (0:len-1) - packet1.angle;
  packets(1) = packet1;
 end
end
