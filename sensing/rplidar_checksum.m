function chk = rplidar_checksum(pkt)
 chk = 0x00;
 for i = 1 : length(pkt)
  chk = bitxor(chk, pkt(i));
 end
end
