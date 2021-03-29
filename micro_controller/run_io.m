% This function handles serial output of the maneuver to the microcontroller.
<<<<<<< HEAD
function [xs, ys, arduinoParams, sensorParams] = run_io(commanding, xpoly, ypoly, arduinoParams, estimatorParams,sensorParams)
=======
function [xs, ys, arduinoParams, sensorParams] = run_io(commanding, xpoly, ypoly, arduinoParams, estimatorParams, sensorParams)
>>>>>>> 14db4d5971739bcb82ac8e74bc65cf8b7b9f1653

if commanding
 send_commands(arduinoParams.arduinoObj, xpoly(2), xpoly(1), ypoly(2), ypoly(1), arduinoParams.xStop, arduinoParams.yStop);
end

 if arduinoParams.arduinoObj.UserData.dataReady
  xs = arduinoParams.arduinoObj.UserData.data.xpos;
  ys = arduinoParams.arduinoObj.UserData.data.ypos;
  arduinoParams.arduinoObj.UserData.dataReady = false;
 else
  xs = estimatorParams.xs;
  ys = estimatorParams.ys;
 end
 
diffx = sensorParams.xoffset - xs;
diffy = sensorParams.yoffset - ys;


sensorParams.boundingbox.cornerx = sensorParams.boundingbox.cornerx - diffx*1000;
sensorParams.boundingbox.cornery = sensorParams.boundingbox.cornery - diffy*1000;
 
end
