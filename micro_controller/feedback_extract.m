clear
clc
close all

blah()
function blah()
filename = "encoder2.txt";
writematrix(["xPos","yPos", "nr"], filename);

arduinoParams = struct;
arduinoParams.readsize = 4;
arduinoParams.portstr = "COM5";
arduinoParams.ratio = 0.0000705;

arduinoParams.arduinoObj = serial_Arduino(arduinoParams);
cleanup = onCleanup(@()clean_up(arduinoParams.arduinoObj));
pause(75)
%% Send manuever commands
send_commands(arduinoParams.arduinoObj,0.0,0.000,0.0,0.000,0.2,0.2);

%%
while true
 pause(1e-6)
 if arduinoParams.arduinoObj.UserData.dataReady && isstruct(arduinoParams.arduinoObj.UserData.feedback)
  writematrix([arduinoParams.arduinoObj.UserData.feedback.xpos', arduinoParams.arduinoObj.UserData.feedback.ypos', arduinoParams.arduinoObj.UserData.nr], filename, 'WriteMode', 'append');
  arduinoParams.arduinoObj.UserData.dataReady = false;
 end
end
end

function clean_up(arduinoObj)
 delete(arduinoObj);
end
