function arduinoObj = serialArduino
%Handles setting up Arduino serial object and also defines callback function
%to read gantry data from Arduino
messageLength = 16/16; %unit is in int8 length, each measurement will be 8 bytes based on two's compliment arithmetic for +/- 500mm in x and y dir, if we process data in MATLAB, data will be in different form
%readSize = messageLength*1; %how many messages to read at once
arduinoObj = serialport("COM4",9600);
arduinoObj.UserData = struct('dataReady', false, 'positionX', 0,'positionY', 0, 'totalRead', 0, 'alert', false);
configureTerminator(arduinoObj,"CR/LF"); %don't really need this if we aren't using readline I don't think
flush(arduinoObj); %clear stream

%% Interrupt function

function arduinoInterrupt(arduinoObj,~)
messageType = read(sensorObj, 2, "uint8")
if messageType == 255 %this is used to signify a measurement package
    while arduinoObj.BytesAvailable > 2 %two bytes for terminator I think?
        arduinoObj.UserData.positionX = [arduinoObj.UserData.positionX, read(arduinoObj, messageLength, "int8")'];
        arduinoObj.UserData.positionY = [arduinoObj.UserData.positionY, read(arduinoObj, messageLength, "int8")'];
    end
    arduinoObj.UserData.dataReady = true;
elseif messageType == 0 %this is used when ascii message is sent from Arduino
    % process arduino message 
    arduinoObj.UserData.alert = true; 
else
    %data has been processed wrong... tell arduino to stop sending data and flush stream 
end


configureCallback(sensorObj, "terminator", @arduinoInterrupt)%might be a better idea to set this to a terminator type then code terminator into ardunio messages

end


end
