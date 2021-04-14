function [t, m] = make_sensor_readings(sim, sensor)
 cont = true;
 i = 1;
 while cont
  cont = advance(sim);
  tg = targetMeshes(sim.Platforms{1});
  release(sensor);
  sensor.MountingAngles = set_orientation(tg);
  t(i) = sim.SimulationTime;
  pts = sensor(tg, t(i));
  m(i, :) = mean(reshape(pts(~isnan(pts)), size(pts)));
  i = i + 1;
 end
end
  