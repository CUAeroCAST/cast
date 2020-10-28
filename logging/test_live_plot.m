% Test the functionality of live plot function
close all;clc;clear all;

%% Dummy Variable Setup
incomingPosY = [20:-.5:0]';
incomingPosX = [20:-.5:0]';

gantryPosX = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];
gantryPosY = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;.5;1;1.5;2;2.5;3;3.5;4;4.5;5];

covRadX = rand(42,1)*.5+1;
covRadY = rand(42,1)*.5+1;

estimatorParams = 0;

PlotStruct.covEllipses = [0 0]; %covAxes must be 1x2 where first el is yrad, second is zrad
PlotStruct.incomingPositions = [20 20]; %est pos is 1x2
PlotStruct.gantryPositions = [0 0]; %gantryPos is 1x2

%% Video setup
filename = '2D_collision_avoid_test';
vobj = VideoWriter(filename, 'MPEG-4');
vobj.FrameRate = 2;
vobj.Quality = 100;
open(vobj);
videoFig = figure;
axis = [-1,20,-1,20];

%% Test function
colOccurance = 0;   %flag for if covariance has intersected with gantry pos
for i = 1:length(incomingPosY)
    estimate.corrState = [incomingPosX(i),0,incomingPosY(i),0,0,0];
    estimate.Pcorr = [covRadX(i) 0 0 0 0 0;0 0 0 0 0 0;0 0 covRadY(i) 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0];
    recv.state = [gantryPosX(i),gantryPosY(i)];
   [PlotStruct,colOccurance] = update_live_plot(PlotStruct,estimate...
       ,recv,vobj,axis,colOccurance,i);
end
close(vobj)
close(videoFig)