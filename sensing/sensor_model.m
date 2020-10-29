% This function operates a parametric sensor model, returning the readings from a relative
% orbit.
function sensorReadings = sensor_model(scenario)
n_steps = scenario.UpdateRate*(scenario.StopTime - scenario.SimulationTime);
%Store sensor readings in cell matrix as we don't know the # of channels
sensorReadings = cell(n_steps,1);


if(length(scenario.Platforms) > 2)
    warning("More than 2 platforms in sensor scenario")
end
ego = scenario.Platforms{1}; %Take first platform as ego platform
if(length(ego.Sensors) > 1)
    warning("More than 1 sensor on ego platform, resortting to first sensor");
end
sensor = ego.Sensors{1};

for i = 1:n_steps 
    advance(scenario);
    tgtmeshes = targetMeshes(ego);
    sensorReadings{i} = sensor(tgtmeshes,pose(ego),scenario.SimulationTime);
end
end