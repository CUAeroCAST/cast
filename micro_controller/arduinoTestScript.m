% Test Arduino read/write functionality

%% Housekeeping
clear all; clc; close all;

%% Set up serial object
arduinoObj = serialArduino;
%configureCallback(arduinoObj,"off");
%% Start Reading Stream
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


