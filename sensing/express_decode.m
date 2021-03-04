function packet = express_decode(bits)

 syncShift = -4;
 startAngleShift = 8;
 startAngleReduce = 64;
 distanceShift = [6, -2];
 angleShift = 4;
 angleReduce = 8;
 sample1Start = 0;
 sample2Start = 2;

 headerSize = 4;
 cabinSize = 5;
 
 packet = struct;
 len = (length(bits) - headerSize) / cabinSize;
 packet.distance = zeros(1, len);
 packet.angle = zeros(1, len);
 
 packet.sync = bitand(bits(1), 0xF0) + bitshift(bits(2), syncShift);
 packet.recvxsum = bitshift(bitand(bits(2), 0xF), -syncShift) + bitand(bits(1), 0xF);
 packet.xsum = rplidar_checksum(bits(3:end));
 packet.startAngle = double((promote_shift(bitand(bits(4), 0x7F), startAngleShift) + uint16(bits(3)))) / startAngleReduce;
 packet.start = bitshift(bitand(bits(4), 0x80), -7);
 
 sample = 1;
 for i = 5:length(bits)-1
%   if mod(i-5, 2) == 0
%    packet.distance(sample) = promote_shift(bits(i), 8) + uint16(bits(i+1));
%   end
  if mod(i, 5) == sample1Start
   packet.distance(sample) = promote_shift(bits(i+1), distanceShift(1)) + promote_shift(bits(i), distanceShift(2));
   packet.angle(sample) = double(bitshift(bitand(bits(i), 0x03), angleShift) + bitand(bits(i+4), 0x0F)) / angleReduce;
   sample = sample + 1;
  elseif mod(i, 5) == sample2Start
   packet.distance(sample) = promote_shift(bits(i+1), distanceShift(1)) + promote_shift(bits(i), distanceShift(2));
   packet.angle(sample) = double((bitshift(bitand(bits(i), 0x03), angleShift) + bitshift(bits(i+2), -angleShift))) / angleReduce;
   sample = sample + 1;
  end
 end
 packet.distance = double(packet.distance);
 packet.angle(packet.angle>360) = packet.angle(packet.angle>360) - 360;
 packet.angle(packet.angle<0) = packet.angle(packet.angle<0) + 360;
end
