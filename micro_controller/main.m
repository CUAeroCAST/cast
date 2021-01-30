% ULA COLLISION AVOINDANCE TESTBED
% ARDUINO COMMUNICATION INTERFACE WITH MATLAB
% AUTHOR: Griffin Van Anne
% DATE: 1/19/21
%% Housekeeping
clear all; clc; close all;

%% Set up serial object
arduinoObj = serialport("COM4",9600)

configureTerminator(arduinoObj,"CR/LF"); %believe this is what we want for terminating in windows

flush(arduinoObj); %clear stream

arduinoObj.UserData = struct("Data",[],"Count",1);

%% Start Reading Stream
figure
hold on
x = 0;
for i = 1:10
    write(arduinoObj,x,"uint8");
    plot(i,x,'rx','linewidth', 1.5)
    x = readline(arduinoObj);
    x = str2double(x);
    plot(i+1,x,'bx','linewidth', 1.5)
end

delete(arduinoObj)


