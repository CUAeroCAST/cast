function params = make_plotting_params()
global datapath;
params.filename = [datapath, filesep, '2D_collision_avoid'];
params.vobj = VideoWriter(params.filename, 'MPEG-4');
params.vobj.Quality = 100;
open(params.vobj);
if hideFig
    params.videoFig = figure('visible', 'off');
else
    params.videoFig = figure;    
end
params.collisionFlag = 0;   %flag for if covariance has intersected with gantry pos
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
params.interval = 50; %how often the plot is updated... if equals 0, then it plots every iteration
params.axis = [-.5 2 -1.25 1.25];
end
