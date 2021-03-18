% This function handles serial output of the maneuver to the microcontroller.
function [xs, ys, arduinoParams] = run_io(xpoly, ypoly, arduinoParams, estimatorParams)
 if arduinoParams.arduinoObj.UserData.dataReady
  xs = arduinoParams.arduinoObj.UserData.data.xpos;
  ys = arduinoParams.arduinoObj.UserData.data.ypos;
  arduinoParams.arduinoObj.UserData.dataReady = false;
 else
  xs = estimatorParams.xs;
  ys = estimatorParams.ys;
 end
 send_commands(arduinoParams.arduinoObj, xpoly(2), xpoly(1), ypoly(2), ypoly(1), arduinoParams.xStop, arduinoParams.yStop);
end
