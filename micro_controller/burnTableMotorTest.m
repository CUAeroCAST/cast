close all; clc;
clearvars -except burnTable timeTable chiefOrbit tChief
% arduinoObj = serialport('COM4',9600);
addpath('C:\Users\Jason\Documents\ASEN4018-4028\LookupTables')
addpath(genpath('../'))
load('burnTable.mat');
load('timeTable.mat');
load('tChief.mat');
load('chiefOrbit.mat');
timeVec = 0:2.004e-4:5;
burnTime = randi(30);
burnDirection = randi(360);
stateAfter = burnTable{burnTime,burnDirection};
tAfter = timeTable{burnTime,burnDirection};
[tAfter,maneuverPos] = convert_2d(tAfter,chiefOrbit(:,1:6),stateAfter(:,1:6));
xPolynomial = polyfit(tAfter,maneuverPos(1,:),4);
yPolynomial = polyfit(tAfter,maneuverPos(2,:),4);
vxPolynomial = polyfit(tAfter,maneuverPos(3,:),4);
vyPolynomial = polyfit(tAfter,maneuverPos(4,:),4);
% sendData(arduinoObj,xPolynomial,yPolynomial,vxPolynomial,vyPolynomial,tMove(1))