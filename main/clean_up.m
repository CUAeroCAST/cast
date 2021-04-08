function clean_up(serialObj, motorStop)
 if motorStop
  stop_Motor(serialObj);
 end
 delete(serialObj);
end