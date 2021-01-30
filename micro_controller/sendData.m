function [status] = sendData(ardunioObj,message)
%Function for sending data to Arduino
%   Sends floating point message to Arduino. Returns 1 for successful
%   write, returns 0 for unsuccessful write. Message is set up to be a
%   vector

%% TODO
% Determine message we are sending, floating point and what precision
% 

%% Condition Message
% Translate floating point digits to ascii charaters to be sent to Arduino


% Might be a better way of doing this without a loop, I couldn't find any
for i = 1:length(message) 
    packet = str(message(i));
    write(arduinoObj,packet,"ascii");
end

% Message has been sent, now send line terminators
write(arduinoObj,'\r\n',"ascii");


outputArg1 = inputArg1;
outputArg2 = inputArg2;
end

