clear;close all;clc
% If this makes it into a pull request, it doesn't need to go in main
importCast
load('positionData2.mat');
covMat = [.000000014 9.9998e-8 0 0 0 0;
    9.9998e-8 .000000014 0 0 0 0;
    0 0 .000000014 9.9998e-8 0 0;
    0 0 9.9998e-8 .000000014 0 0;
    0 0 0 0 .000000014 9.9998e-8;
    0 0 0 0 9.9998e-8 .000000014];
vSat = sqrt(398600/7578);
satellite = [satellite zeros(length(satellite),3)];
for i=1:length(satellite)
    if satellite(i,1)>0
        theta = atan(satellite(i,3)/satellite(i,1));
        velUnit = [-sin(theta) 0 cos(theta)];
    else
        theta = atan(satellite(i,3)/-satellite(i,1));
        velUnit = [-sin(theta) 0 -cos(theta)];
    end
    vel = vSat*velUnit;
    satellite(i,4:6) = vel;
end
ind = 70326;
satelliteState = satellite(70326,:);
corrState = hill(100326,:);
predictedState.corrState = [corrState(1) 0 corrState(2) 0 corrState(3) 0];
predictedState.Pcorr = covMat;
man = make_maneuver(predictedState,satelliteState,30);