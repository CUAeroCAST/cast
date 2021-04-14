function [sim, sensor, initState] = make_simulation(t, chiefOrbit, collisionOrbit, simParams)
 sim = trackingScenario("UpdateRate", simParams.rate, "StopTime", t(end));
 chief = platform(sim);
 deputy = platform(sim);
 deputy.Mesh = extendedObjectMesh("cuboid");
 deputy.Mesh = scale(deputy.Mesh, 1000);
 
 relPos = collisionOrbit(1, 1:3) - chiefOrbit(1, 1:3);
 relVel = collisionOrbit(1, 4:6) - chiefOrbit(1, 4:6);
 
 along = chiefOrbit(1, 4:6) / norm(chiefOrbit(1, 4:6));
 rad = chiefOrbit(1, 1:3) / norm(chiefOrbit(1, 1:3));
 cross_ = cross(rad, along);
 
 initState = [dot(along, relPos);
              dot(cross_, relPos);
              dot(along, relVel);
              dot(cross_, relVel)];
 range = max(vecnorm(relPos, 2, 2)) + 1;
 
 chief.Trajectory = waypointTrajectory(chiefOrbit(:, 1:3), t, "Velocities", chiefOrbit(:, 4:6));
 deputy.Trajectory = waypointTrajectory(collisionOrbit(:, 1:3), t, "Velocities", collisionOrbit(:, 4:6));
 tg = targetMeshes(chief);
 orien = set_orientation(tg);
 sensor = monostaticLidarSensor(1, "UpdateRate", simParams.rate,...
          "MaxRange", range, "RangeAccuracy", simParams.acc, "ElevationLimits", simParams.ebounds,...
          "AzimuthLimits", simParams.abounds,   "AzimuthResolution", simParams.res, "ElevationResolution", simParams.res,...
          "MountingAngles", orien);
end
 