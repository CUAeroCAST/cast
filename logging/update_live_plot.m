% This function updates a live plot during runtime

function plotStruct = update_live_plot(plotStruct,estimate,recv,i)
videoObj = plotStruct.vobj;
axis_ = plotStruct.axis;
%% Extract information from estimate struct and recv struct
covAxes = [estimate.Pcorr(1,1),estimate.Pcorr(3,3)]; %This may need refining if off-axes tilt for ellipse is present
estPos = [estimate.corrState(1),estimate.corrState(3)];    %x and y estimated position for incoming object
gantryPos = [recv.state(1),recv.state(2)];         %assume for now that recv.state is [xpos,ypos]

%% Append New Data into PlotStruct
if isfield(plotStruct,'covEllipses')%test if this field has been created yet
    %structure fields: 'covEllipses', 'incomingPositions', 'incomingEstPositions', 'gantryPositions'
    plotStruct.covEllipses = [plotStruct.covEllipses;covAxes]; %covAxes must be 1x2 where first el is yrad, second is zrad
    plotStruct.incomingPositions = [plotStruct.incomingPositions;estPos]; %est pos is 1x2
    plotStruct.gantryPositions = [plotStruct.gantryPositions;gantryPos]; %gantryPos is 1x2
    %Delete plots from previous step
    set(groot,'defaultLegendAutoUpdate','off')
    for i = 1:length(plotStruct.plots2Delete)
        delete(plotStruct.plots2Delete(i)) 
    end
else
    plotStruct.covEllipses = covAxes; %covAxes must be 1x2 where first el is yrad, second is zrad
    plotStruct.incomingPositions = estPos; %est pos is 1x2
    plotStruct.gantryPositions = gantryPos; %gantryPos is 1x2
    %legend('Incoming Object Covariance','Incoming Object Path','Gantry Path','Gantry Position','Estimated Incoming Object Position','location','northwest')
    title('2D Collision Live Scenario')
    xlabel('X distance(m)')
    ylabel('Y distance(m)')
    axis(axis_)
end

%% Plot updated path with current positional and covariance information

hold on
plotDel1 = ellipse(estPos(1),estPos(2),2*covAxes(1),2*covAxes(2),'color','r','linewidth',2);
plot(plotStruct.incomingPositions(:,1),plotStruct.incomingPositions(:,2),'color','r','linewidth',2);
plot(plotStruct.gantryPositions(:,1),plotStruct.gantryPositions(:,2),'color','b','linewidth',2);
plotDel2 = plot(gantryPos(1),gantryPos(2),'b*','linewidth',2);
plotDel3 = plot(estPos(1),estPos(2),'r*','linewidth',2);
%delete elements 4,5,1 each step
hold off
if length(plotStruct.gantryPositions(:,1))==1
    legend('Incoming Object Covariance','Incoming Object Path','Gantry Path','Gantry Position','Estimated Incoming Object Position','location','northwest','AutoUpdate','off')
end
% Add warning to show when manuevor starts
if length(plotStruct.gantryPositions(:,1))>1
    if ((gantryPos(1)~=plotStruct.gantryPositions(end-1,1)) || (gantryPos(2)~=plotStruct.gantryPositions(end-1,2)))
        txt = ('Spacecraft is Maneuvering');
        t1 = annotation('textbox');
        t1.String = txt;
        t1.BackgroundColor = [0.4660 0.6740 0.1880];
        t1.Position = [.7 .15 .12 .04];
    end
end
% Add confirmation if object has been avoided
if  plotStruct.collisionFlag == 0
    if ((estPos(1)-2*covAxes(1))<=gantryPos(1) &&   (estPos(1)+2*covAxes(1))>=gantryPos(1)...
            && (estPos(2)-2*covAxes(2))<=gantryPos(2) &&   (estPos(2)+2*covAxes(2))>=gantryPos(2))
        plotStruct.collisionFlag = 1;
    end
elseif plotStruct.collisionFlag == 1
        txt = ('Potential Collision Has Occurred!');
        t2 = annotation('textbox');
        t2.String = txt;
        t2.BackgroundColor = [1 0 0];
        t2.Position = [.7 .25 .15 .04];
end

%% Save frame to video

frame = getframe(gcf);
%legend(gca, 'off');
writeVideo(videoObj, frame);
%cla(gca);
plotStruct.plots2Delete = [plotDel1;plotDel2;plotDel3];

end