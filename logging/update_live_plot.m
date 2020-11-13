% This function updates a live plot during runtime

function plotStruct = update_live_plot(plotStruct,estimate,collisionEstimate,recv)
videoObj = plotStruct.vobj;
axis_ = plotStruct.axis;
%% Extract information from estimate struct and recv struct
if ~isnan(estimate.corrState(1))
    estPos = [estimate.corrState(1),estimate.corrState(3)];    %x and y estimated position for incoming object
    [v,lamCorr] = eig([estimate.Pcorr(1,1) estimate.Pcorr(1,3);estimate.Pcorr(3,1) estimate.Pcorr(3,3)]); %getting eigenstuff from the 2D position state matrix
    covAxes = [2*sqrt(lamCorr(1,1)), 2*sqrt(lamCorr(2,2))]; %major and minor axis of cov
    covTilt = atan(v(2,1)/v(1,1)); %tilt of ellipse in radians
    covTiltCorr = covTilt;
    [v,lamPred] = eig([estimate.Ppred(1,1) estimate.Ppred(1,3);estimate.Ppred(3,1) estimate.Ppred(3,3)]); %getting eigenstuff from the 2D position state matrix    
    covTiltPred = atan(v(2,1)/v(1,1));
else
    estPos = [estimate.predState(1),estimate.predState(3)];    %x and y estimated position for incoming object
    [v,lamPred] = eig([estimate.Ppred(1,1) estimate.Ppred(1,3);estimate.Ppred(3,1) estimate.Ppred(3,3)]); %getting eigenstuff from the 2D position state matrix    
    covAxes = [2*sqrt(lamPred(1,1)), 2*sqrt(lamPred(2,2))]; %major and minor axis of cov
    covTilt = atan(v(2,1)/v(1,1)); %tilt of ellipse in radians 
    covTiltPred = covTilt;
    covTiltCorr = nan; %tilt of ellipse in radians
    lamCorr = [nan,nan;nan,nan];

end
gantryPos = [recv.state(1),recv.state(2)];         %assume for now that recv.state is [xpos,ypos]

%% Append New Data into PlotStruct
if isfield(plotStruct,'covEllipsesCorr')%test if this field has been created yet
    %structure fields: 'covEllipses', 'incomingPositions', 'incomingEstPositions', 'gantryPositions'
    plotStruct.covEllipsesCorr = [plotStruct.covEllipsesCorr;[2*sqrt(lamCorr(1,1)), 2*sqrt(lamCorr(2,2))]]; %covAxes must be 1x2 where first el is yrad, second is zrad
    plotStruct.covEllipsesPred = [plotStruct.covEllipsesPred;[2*sqrt(lamPred(1,1)), 2*sqrt(lamPred(2,2))]]; %covAxes must be 1x2 where first el is yrad, second is zrad
    plotStruct.covTiltCorr = [plotStruct.covTiltCorr;covTiltCorr];
    plotStruct.covTiltPred = [plotStruct.covTiltPred;covTiltPred];
    plotStruct.incomingPositions = [plotStruct.incomingPositions;estPos]; %est pos is 1x2
    plotStruct.gantryPositions = [plotStruct.gantryPositions;gantryPos]; %gantryPos is 1x2
    %Delete plots from previous step
    set(groot,'defaultLegendAutoUpdate','off')
    for i = 1:length(plotStruct.plots2Delete)
        delete(plotStruct.plots2Delete(i)) 
    end
else
    plotStruct.covEllipsesCorr = covAxes; %covAxes must be 1x2 where first el is yrad, second is zrad
    plotStruct.covEllipsesPred = covAxes;
    plotStruct.incomingPositions = estPos; %est pos is 1x2
    plotStruct.gantryPositions = gantryPos; %gantryPos is 1x2
    plotStruct.covTiltCorr = covTiltCorr;
    plotStruct.covTiltPred = covTiltCorr;    
    %legend('Incoming Object Covariance','Incoming Object Path','Gantry Path','Gantry Position','Estimated Incoming Object Position','location','northwest')
    title('2D Collision Live Scenario')
    xlabel('X distance(m)')
    ylabel('Y distance(m)')
    axis(axis_)
end

%% Plot updated path with current positional and covariance information

hold on
plotDel1 = ellipse(estPos(1),estPos(2),covAxes(1),covAxes(2),covTilt,'color','m','linewidth',2);
plot(plotStruct.incomingPositions(:,1),plotStruct.incomingPositions(:,2),'color','r','linewidth',2);
plot(plotStruct.gantryPositions(:,1),plotStruct.gantryPositions(:,2),'color','b','linewidth',2);
plotDel2 = plot(gantryPos(1),gantryPos(2),'b*','linewidth',2);
plotDel3 = plot(estPos(1),estPos(2),'r*','linewidth',2);

% Plot cyan ellipse around predicted collision
[v,lam] = eig([collisionEstimate.Ppred(1,1) collisionEstimate.Ppred(1,3);collisionEstimate.Ppred(3,1) collisionEstimate.Ppred(3,3)]); %getting eigenstuff from the 2D position state matrix
covAxes = [2*sqrt(lam(1,1)), 2*sqrt(lam(2,2))]; %major and minor axis of cov
covTilt = atan(v(2,1)/v(1,1)); %tilt of ellipse in radians
plotDel4 = ellipse(collisionEstimate.predState(1),collisionEstimate.predState(3),...
    covAxes(1),covAxes(2),covTilt,'linewidth',2,'color','c');%plot 2sig predicted collision cov

I = ~isnan(plotStruct.covEllipsesCorr(:,2)); %index where corr states exist
plot(plotStruct.incomingPositions(I,1)-sin(-plotStruct.covTiltCorr(I)).*plotStruct.covEllipsesCorr(I,1),plotStruct.incomingPositions(I,2)+(plotStruct.covEllipsesCorr(I,2).*cos(-plotStruct.covTiltCorr(I))),'--','color','m','linewidth',1)%x1,y1
plot(plotStruct.incomingPositions(:,1)-sin(-plotStruct.covTiltPred(:)).*plotStruct.covEllipsesPred(:,1),plotStruct.incomingPositions(:,2)+(plotStruct.covEllipsesPred(:,2).*cos(-plotStruct.covTiltPred(:))),'--','color','k','linewidth',1)%x1,y1
plot(plotStruct.incomingPositions(:,1)+sin(-plotStruct.covTiltPred(:)).*plotStruct.covEllipsesPred(:,1),plotStruct.incomingPositions(:,2)-(plotStruct.covEllipsesPred(:,2).*cos(-plotStruct.covTiltPred(:))),'--','color','k','linewidth',1)%x2,y2
plot(plotStruct.incomingPositions(I,1)+sin(-plotStruct.covTiltCorr(I)).*plotStruct.covEllipsesCorr(I,1),plotStruct.incomingPositions(I,2)-(plotStruct.covEllipsesCorr(I,2).*cos(-plotStruct.covTiltCorr(I))),'--','color','m','linewidth',1)

%delete elements 4,5,1 each step
hold off
if length(plotStruct.gantryPositions(:,1))==1
    legend('Incoming Object 2\sigma Covariance','Incoming Object Path','Gantry Path','Gantry Position','Estimated Incoming Object Position','Predicted Collision 2\sigma Covariance','2\sigma Corrected Covariance','2\sigma Predicted Covariance','location','northwest','AutoUpdate','off')
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
    if ((estPos(1)-(covAxes(1)))<=gantryPos(1) &&   (estPos(1)+(covAxes(1)))>=gantryPos(1)...
            && (estPos(2)-(covAxes(2)))<=gantryPos(2) &&   (estPos(2)+(covAxes(2)))>=gantryPos(2))...
            && collisionEstimate.collisionTime<50
        plotStruct.collisionFlag = 1;
    end
elseif plotStruct.collisionFlag == 1
        txt = ('Potential Collision Has Occurred!');
        t2 = annotation('textbox');
        t2.String = txt;
        t2.BackgroundColor = [1 0 0];
        t2.Position = [.7 .25 .15 .04];
end
% Add confirmation if gantry can manuever outside predicted covariance
if (-2*sqrt((collisionEstimate.Ppred(1,1)))>-.5 ||   2*sqrt((collisionEstimate.Ppred(1,1)))<.5... %is cov small enough to fit within gantry
            || (-2*sqrt((collisionEstimate.Ppred(3,3))))>-.5 ||   (2*sqrt((collisionEstimate.Ppred(3,3))))<.5)
        if (2*sqrt(min([collisionEstimate.Ppred(3,3), collisionEstimate.Ppred(1,1)]))/collisionEstimate.collisionTime+.1) < .5 %is gantry fast enough to get outside before collisiontime
            txt = ('Gantry Manuever Outside Collision Prediction Covariance Possible!');
            t1 = annotation('textbox');
            t1.String = txt;
            t1.BackgroundColor = [.1615 0.3807 0.7346];
            t1.Position = [.7 .35 .15 .08];
        end
end

%% Save frame to video

frame = getframe(gcf);
%legend(gca, 'off');
writeVideo(videoObj, frame);
%cla(gca);
plotStruct.plots2Delete = [plotDel1;plotDel2;plotDel3;plotDel4];

end