% This function updates a live plot during runtimee
function PlotStruct = update_live_plot(PlotStruct,estimatorParams,gantryPos,covAxes,estPos,videoObj,f,AXIS)

%% Extract information from estimator params
%implement later once this struct is defined
%covAxes = ...
%est

%% Append New Data into PlotStruct
%structure fields: 'covEllipses', 'incomingPositions', 'incomingEstPositions', 'gantryPositions'
PlotStruct.covEllipses = [PlotStruct.covEllipses;covAxes]; %covAxes must be 1x2 where first el is yrad, second is zrad
PlotStruct.incomingPositions = [PlotStruct.incomingPositions;estPos]; %est pos is 1x2
PlotStruct.gantryPositions = [PlotStruct.gantryPositions;gantryPos]; %gantryPos is 1x2

%% Plot updated path with current positional and covariance information
hold on
ellipse(estPos(1),estPos(2),2*covAxes(1),2*covAxes(2),'color','r','linewidth',2)
plot(PlotStruct.incomingPositions(:,1),PlotStruct.incomingPositions(:,2),'linewidth',2)
plot(PlotStruct.gantryPositions(:,1),PlotStruct.gantryPositions(:,2),'linewidth',2)
axis(AXIS)
plot(gantryPos(1),gantryPos(2),'b*','linewidth',2)
plot(estPos(1),estPos(2),'r*','linewidth',2)
legend('Incoming Object Covariance','Incoming Object Path','Gantry Path','Gantry Position','Estimated Incoming Object Position','location','northwest')
title('2D Collision Live Scenario')
xlabel('X distance(m)')
ylabel('Y distance(m)')
%% Save frame to video
frame = getframe(gcf);
legend(gca, 'off')
writeVideo(videoObj, frame);
cla(gca);
end