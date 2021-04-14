function [sim, sensor, initState] = make_simulation(t, chiefOrbit, collisionOrbit, simParams)
 sim = trackingScenario("UpdateRate", simParams.rate, "StopTime", t(end));
 chief = platform(sim);
 deputy = platform(sim);
 deputy.Mesh = extendedObjectMesh("cuboid");
 deputy.Mesh = scale(deputy.Mesh, 1000);
 
 relPath = collisionOrbit(:, 1:3) - chiefOrbit(:, 1:3);
 relVel = collisionOrbit(:, 4:6) - chiefOrbit(:, 4:6);
 relMag = norm(relPath(1, :));
 relVMag = dot(relVel(:,1), relPath(:,1)) / relMag;
 initState = [relMag; 0; relVMag; 0];
 range = max(vecnorm(relPath, 2, 2)) + 1;
 
 chief.Trajectory = waypointTrajectory(chiefOrbit(:, 1:3), t, "Velocities", chiefOrbit(:, 4:6));
 deputy.Trajectory = waypointTrajectory(collisionOrbit(:, 1:3), t, "Velocities", collisionOrbit(:, 4:6));
 tg = targetMeshes(chief);
 orien = set_orientation(tg);
 sensor = monostaticLidarSensor(1, "UpdateRate", simParams.rate,...
          "MaxRange", range, "RangeAccuracy", simParams.acc, "ElevationLimits", simParams.ebounds,...
          "AzimuthLimits", simParams.abounds,   "AzimuthResolution", simParams.res, "ElevationResolution", simParams.res,...
          "MountingAngles", orien);
 