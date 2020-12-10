% This script makes the lookup table
clear;close all;clc;
chiefState = [0;0;7578;7.25256299066873;0;0;850]';
mu = 398600;
burnTable = cell(30,360);
timeTable = cell(30,360);
positionTable = cell(30,360);
timeCell = cell(30,1);
endPos = [217.547021908829,0,7574.8767210696];
numTimesteps = 2753;
tspan = linspace(0,30,numTimesteps);
[tAfter,stateAfter] =  ode45(@(tAfter, stateAfter) orbit_prop_maneuver(tAfter,...
    stateAfter,mu,[1 0 0]',0),tspan,chiefState);
% a = zeros(3,length(tAfter));
% for i = 1:length(stateAfter)
%     [~,avec] = orbit_prop_maneuver(tAfter(i),stateAfter(i,:)',mu,[1 0 0]',0);
%     a(:,i) = avec;
% end
burnStruct.tAfter = tAfter;
burnStruct.stateAfter = stateAfter;
% burnStruct.a = a;
for i = 1:360
    for j = 1:30
        burnTable{j,i} = stateAfter;
        timeTable{j,i} = tAfter;
        positionTable{j,i} = endPos;
    end
end
burnTime = 1;
for i = 1:30
    [tAfter,stateAfter] =  ode45(@(tAfter, stateAfter) orbit_prop_maneuver(tAfter,...
    stateAfter,mu,[-0.025543488617113;-0.009503141266992;0],burnTime),tspan,chiefState);
%     for j = 1:length(stateAfter)
%         [~,avec] = orbit_prop_maneuver(tAfter(j),stateAfter(j,:)',mu,...
%             [-0.025543488617113;-0.009503141266992;0],burnTime);
%         a(:,j) = avec;
%     end
    endPos = stateAfter(end,1:3);
    burnStruct.tAfter = tAfter;
    burnStruct.stateAfter = stateAfter;
%     burnStruct.a = a;
    burnTable{i,200} = stateAfter;
    timeTable{i,200} = tAfter;
    positionTable{i,200} = endPos;
    burnTime = burnTime+1;
end