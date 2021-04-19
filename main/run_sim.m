clear
clc
close all
importCast;

global datapath;
datapath = open_logging(true);

estimatorParams = make_estimator_params();
estimatorParams.currentTime = 0;
guidanceParams = make_guidance_params();
simParams = make_sim_params();

[t, chiefOrbit, collisionOrbit] = generateOrbits(simParams, guidanceParams);
guidanceParams.colState = chiefOrbit(end, :)';
[sim, sensor, estimatorParams.filter.State] = make_simulation(t, chiefOrbit, collisionOrbit, simParams);
[t, m] = make_sensor_readings(sim, sensor, simParams);

log_struct(estimatorParams, [datapath, filesep, 'estimatorParams'])
log_struct(guidanceParams, [datapath, filesep, 'guidanceParams'])
log_struct(simParams, [datapath, filesep, 'simParams'])

collisionEstimate(1) = struct("collisionTime", nan, "predState", nan, "Ppred", nan);

steps = length(t);
estimateStorage(steps) = struct("corrState", nan, "Pcorr", nan, "predState", nan, "Ppred", nan);
collisionStorage(steps) = struct("collisionTime", nan, "predState", nan, "Ppred", nan);
measurementStorage(steps) = struct("time", nan, "distance", nan, "angle", nan);

moving = false;
for i = 1 : steps
 measurement.time = t(i);
 measurement.distance = m(i, 1);
 measurement.angle = m(i, 2);
 
 [estimate, estimatorParams] = state_estimator([measurement.distance; measurement.angle],...
                                        t(i), estimatorParams);
 collisionEstimate = collision_prediction(estimate, estimatorParams, collisionEstimate);
 if ~moving
  timeToCol = t(end) - t(i) + 0.2;%collisionEstimate.collisionTime - t(i);
  [maneuver, tManeuver, stateManeuver] = make_maneuver_sim(collisionEstimate, [chiefOrbit(i,:), simParams.mass], timeToCol, guidanceParams);
  if maneuver(3)
   moving = true;
   n = i;
  end
 end
 estimateStorage(i) = estimate;
 collisionStorage(i) = collisionEstimate;
 measurementStorage(i) = measurement;
end

estimate = collisionStorage(n);
estimate.Pcorr = estimate.Ppred;
estimate.corrState = estimate.predState;
xmax = estimate.corrState(1)+2*sqrt(estimate.Pcorr(1,1));
xmin =  estimate.corrState(1)-2*sqrt(estimate.Pcorr(1,1));
ymax = estimate.corrState(2)+2*sqrt(estimate.Pcorr(2,2));
ymin =  estimate.corrState(2)-2*sqrt(estimate.Pcorr(2,2));
satelliteState = chiefOrbit(n, :);
colState = chiefOrbit(end, :);
unitRad = satelliteState(1:3)/norm(satelliteState(1:3));
unitAlong = satelliteState(4:6)/norm(satelliteState(4:6));
unitCross = cross(unitRad,unitAlong);
Q = [unitAlong;unitCross;unitRad]';
minvecTemp = Q * [xmin; ymin; 0];
maxvecTemp = Q * [xmax; ymax; 0];
maxvec = max([minvecTemp'; maxvecTemp']);
minvec = min([minvecTemp'; maxvecTemp']);

[X, Y, Z] = sphere;

X = X*maxvec(1) + colState(1);
Y = Y*maxvec(2) + colState(2);
Z = Z*maxvec(3) + colState(3);
plot3(chiefOrbit(:,1), chiefOrbit(:,2), chiefOrbit(:,3), "LineWidth", 1)
hold on
plot3(collisionOrbit(:,1), collisionOrbit(:,2), collisionOrbit(:,3), "LineWidth", 1)
plot3(stateManeuver(:,1), stateManeuver(:,2), stateManeuver(:,3), "LineWidth", 1)
s = surf(X,Y,Z);
set(s, "FaceColor", [0,0,0]);
set(s, "FaceAlpha", 0.25)
legend("Before Maneuver", "Space Junk", "After Maneuver", "2\sigma Ellipse")
grid minor
xlabel("ECEF-X (m)")
ylabel("ECEF-Y (m)")
zlabel("ECEF-Z (m)")
title("Maneuver Comparison Integrated to Time-of-Collision")
xticks([-100, -50, 0, 50, 100])
yticks([-50, 0, 50])
zticks([7578e3 - 10, 7578e3, 7578e3 + 10])

log_struct(estimateStorage, [datapath, filesep, 'estimateStorage'])
log_struct(collisionStorage, [datapath, filesep, 'collisionStorage'])
log_struct(measurementStorage, [datapath, filesep, 'measurementStorage'])
log_struct(maneuver, [datapath, filesep, 'maneuver'])
log_struct(stateManeuver, [datapath, filesep, 'maneuveredOrbit'])
log_struct(chiefOrbit, [datapath, filesep, 'chiefOrbitsim'])
log_struct(collisionOrbit, [datapath, filesep, 'collisionOrbitsim'])
