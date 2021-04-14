function [t, m] = make_sensor_readings(sim, sensor, simParams)
 n = simParams.time * simParams.rate;
 t(n) = nan;
 m(n, 2) = nan;
 for i = 1 : n
  advance(sim);
  tg = targetMeshes(sim.Platforms{1});
  release(sensor);
  sensor.MountingAngles = set_orientation(tg);
  t(i) = sim.SimulationTime;
  theta = 180 + sensor.MountingAngles(1);
  pts = sensor(tg, t(i));
  m(i, :) = [norm(mean(reshape(pts(~isnan(pts)), size(pts)))), theta];
 end
end
  