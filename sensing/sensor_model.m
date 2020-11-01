% This function operates a parametric sensor model, returning the readings from a relative
% orbit.
function sensorReadings = sensor_model(scenario, makePlot)
n_steps = floor(scenario.UpdateRate*(scenario.StopTime - scenario.SimulationTime));
%Store sensor readings in cell matrix as we don't know the # of channels
sensorReadings = zeros(n_steps,3);


if(length(scenario.Platforms) > 2)
    warning("More than 2 platforms in sensor scenario")
end
ego = scenario.Platforms{1}; %Take first platform as ego platform
if(length(ego.Sensors) > 1)
    warning("More than 1 sensor on ego platform, resortting to first sensor");
end
sensor = ego.Sensors{1};
target = scenario.Platforms{2};

if(makePlot)
    tp = theaterPlot('Ylim', [-5, 5],'ZLim',[-5,5]);
    covp = coveragePlotter(tp,'DisplayName', 'Sensor Coverage');
    op = orientationPlotter(tp, 'DisplayName', 'Sensor Orientation');
    pp = platformPlotter(tp,'DisplayName', 'Target');
    for i = 1:n_steps 
        advance(scenario);
        ps = pose(ego);
        sensorCov = coverageConfig(sensor);
        sensorCov.Orientation = ps.Orientation;
        plotOrientation(op, ps.Orientation)
        plotCoverage(covp,sensorCov,1,{'red'});
        plotPlatform(pp, target.Position);
        tgtmeshes = targetMeshes(ego);
        temp = sensor(tgtmeshes,ps,scenario.SimulationTime);
        if any(isnan(temp),'all')
            %If there are nan values, take the middle reading
            sensorReadings(i,:) = temp(median(1:length(temp(:,1))),:);
        else
            sensorReadings(i,:) = temp;
        end
    end
else
    for i = 1:n_steps 
        advance(scenario);
        tgtmeshes = targetMeshes(ego);
        temp = sensor(tgtmeshes,pose(ego),scenario.SimulationTime);
        if any(isnan(temp),'all')
            %If there are nan values, take the middle reading
            sensorReadings(i,:) = temp(median(1:length(temp(:,1))),:);
        else
            sensorReadings(i,:) = temp;
        end
    end
end