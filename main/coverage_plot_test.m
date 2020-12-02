tp = theaterPlot('XLimits',[0 1500],'Ylimits',[-750, 750],'ZLimits',[-750 750]);
trajPlotter = trajectoryPlotter(tp,'DisplayName','Trajectory');
plotTrajectory(trajPlotter,{relativeOrbit})
targetPlotter = platformPlotter(tp,'DisplayName','Target');
plotPlatform(targetPlotter, sensorScenario.Platforms{1, 2}.Position);
sensorPlotter = coveragePlotter(tp,'DisplayName','Beam','Color','b');
plotCoverage(sensorPlotter,coverageConfig(sensorScenario));