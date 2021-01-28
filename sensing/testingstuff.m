clear
clc
close all
prog = matlabroot;
%!matlab -batch serial_Sensor &
!/Applications/MATLAB_R2020b.app/bin/matlab -batch serial_Sensor &

pause(10)
%%
for i=1:10
 data = get(0, 'userdata');
 if data.readyFlag
  fprintf(data)
  data.readyFlag = false;
  set(0, 'userdata', data);
 end
end

data.running = false;
set(0, 'userdata', data);
