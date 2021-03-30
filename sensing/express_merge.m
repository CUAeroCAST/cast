function [packets, packet2] = express_merge(packet1, bits, numPkts, sensorParams)
 readsize = sensorParams.readsize;
 packets(numPkts - 1) = packet1;
 for i = 1 : numPkts - 1
  packet2 = express_decode(bits(readsize*i + 1: readsize*(i+1)));
  alpha = packet2.startAngle - packet1.startAngle;
  if alpha < 0
   alpha = alpha + 360;
  end
  len = length(packet1.distance);
  packet1.angle = packet1.startAngle + alpha/32 * (0:len-1) - packet1.angle;
  packets(i) = packet1;
  packet1 = packet2;
 end
end
