% This function updates a live plot during runtimee
function [PlotStruct,collisionFlag] = update_live_plot(PlotStruct,estimate,recv,videoObj,AXIS,collisionFlag,i)

%% Extract information from estimate struct and recv struct
covAxes = [estimate.covarianceMat(1,1),estimate.covarianceMat(3,3)]; %This may need refining if off-axes tilt for ellipse is present
estPos = [estimate.state(1),estimate.state(3)];    %x and y estimated position for incoming object
gantryPos = [recv.state(1),recv.state(2)];         %assume for now that recv.state is [xpos,ypos]
%% Append New Data into PlotStruct
if i>1
    %structure fields: 'covEllipses', 'incomingPositions', 'incomingEstPositions', 'gantryPositions'
    PlotStruct.covEllipses = [PlotStruct.covEllipses;covAxes]; %covAxes must be 1x2 where first el is yrad, second is zrad
    PlotStruct.incomingPositions = [PlotStruct.incomingPositions;estPos]; %est pos is 1x2
    PlotStruct.gantryPositions = [PlotStruct.gantryPositions;gantryPos]; %gantryPos is 1x2
else
    PlotStruct.covEllipses = covAxes; %covAxes must be 1x2 where first el is yrad, second is zrad
    PlotStruct.incomingPositions = estPos; %est pos is 1x2
    PlotStruct.gantryPositions = gantryPos; %gantryPos is 1x2
end
%% Plot updated path with current positional and covariance information
hold on
ellipse(estPos(1),estPos(2),2*covAxes(1),2*covAxes(2),'color','r','linewidth',2);
plot(PlotStruct.incomingPositions(:,1),PlotStruct.incomingPositions(:,2),'linewidth',2)
plot(PlotStruct.gantryPositions(:,1),PlotStruct.gantryPositions(:,2),'linewidth',2)
axis(AXIS)
plot(gantryPos(1),gantryPos(2),'b*','linewidth',2)
plot(estPos(1),estPos(2),'r*','linewidth',2)
legend('Incoming Object Covariance','Incoming Object Path','Gantry Path','Gantry Position','Estimated Incoming Object Position','location','northwest')
title('2D Collision Live Scenario')
xlabel('X distance(m)')
ylabel('Y distance(m)')

% Add warning to show when manuevor starts
if i>1
    if ((gantryPos(1)~=PlotStruct.gantryPositions(end-1,1)) || (gantryPos(2)~=PlotStruct.gantryPositions(end-1,2)))
        txt = ('Spacecraft is Maneuvering');
        t1 = annotation('textbox');
        t1.String = txt;
        t1.BackgroundColor = [0.4660 0.6740 0.1880];
        t1.Position = [.5 .25 .34 .07];
    end
end
% Add confirmation if object has been avoided
if  collisionFlag == 0
    if ((estPos(1)-2*covAxes(1))<=gantryPos(1) &&   (estPos(1)+2*covAxes(1))>=gantryPos(1)...
            && (estPos(2)-2*covAxes(2))<=gantryPos(2) &&   (estPos(2)+2*covAxes(2))>=gantryPos(2))
        collisionFlag = 1;
    end
elseif collisionFlag == 1
        txt = ('Potential Collision Has Occurred!');
        t2 = annotation('textbox');
        t2.String = txt;
        t2.BackgroundColor = [1 0 0];
        t2.Position = [.5 .4 .4 .07];
end

%% Save frame to video
frame = getframe(gcf);
legend(gca, 'off');
writeVideo(videoObj, frame);
cla(gca);
end