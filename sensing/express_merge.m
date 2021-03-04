function [packet1, packet2] = express_merge(packet1, bits)
 packet2 = express_decode(bits);
 alpha = packet2.startAngle - packet1.startAngle;
 if alpha < 0
  alpha = alpha + 360;
 end
 len = length(packet1.distance);
 %packet1.angle = packet1.startAngle + alpha/40 * (0:len-1);
 packet1.angle = packet1.startAngle + alpha/32 * (0:len-1) - packet1.angle;
end
