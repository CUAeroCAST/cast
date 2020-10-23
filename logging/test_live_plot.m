% Test the functionality of live plot function
close all;clc;clear all;

%% Dummy Variable Setup
incomingPosY = [20:-1:0]';
incomingPosX = [20:-1:0]';

gantryPosX = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];
gantryPosY = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;1;2;3;4;5];

covRadX = rand(21,1)*.5+1;
covRadY = rand(21,1)*.5+1;

estimatorParams = 0;

PlotStruct.covEllipses = [0 0]; %covAxes must be 1x2 where first el is yrad, second is zrad
PlotStruct.incomingPositions = [20 20]; %est pos is 1x2
PlotStruct.gantryPositions = [0 0]; %gantryPos is 1x2

%% Video setup
filename = '2D_collision_avoid';
vobj = VideoWriter(filename, 'MPEG-4');
vobj.FrameRate = 2;
vobj.Quality = 100;
open(vobj);
videoFig = figure;
axis = [-1,20,-1,20];

%% Test function
for i = 1:length(incomingPosY)
   PlotStruct = update_live_plot(PlotStruct,estimatorParams,[gantryPosX(i),gantryPosY(i)],[covRadX(i),covRadY(i)]...
       ,[incomingPosX(i),incomingPosY(i)],vobj,videoFig,axis);
end
close(vobj)
close(videoFig)