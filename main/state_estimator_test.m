clear
clc
close all
importCast;
timestep = 2.5e-4;

file = "expressdata--nearmiss2.txt";

datum = readmatrix(file);

len = length(datum);

readings = struct;
readings.distance = datum(1:800, 1);
readings.angle = datum(1:800, 2);
estimatorParams = struct;
estimatorParams.currentTime = 0;
estimatorParams.filter.MeasurementModel = @(x,xs,y,ys)...
    [(2*x - 2*xs)/(2*((x - xs)^2 + (y - ys)^2)^(1/2)),...
    (2*y - 2*ys)/(2*((x - xs)^2 + (y - ys)^2)^(1/2)), 0, 0;
    -(y - ys)/((x - xs)^2*((y - ys)^2/(x - xs)^2 + 1)),...
    1/((x - xs)*((y - ys)^2/(x - xs)^2 + 1)), 0, 0]; %Range bearing model
estimatorParams.filter.MeasurementNoise = [0.025^2,0;0,deg2rad(0.45)^2]; %Range, bearing
estimatorParams.filter.ProcessNoise = @(dt) 0.01*eye(4); %constant process noise
estimatorParams.filter.StateCovariance = eye(4); %Initial estimate covariance
estimatorParams.filter.STM = @(dt) eye(4) + [0,0,1,0;0,0,0,1;0,0,0,0;0,0,0,0]*dt;
estimatorParams.filter.State = [1.5;0;-1;0];
collisionEstimate(1) = struct("collisionTime", nan, "predState", nan, "Ppred", nan);

sensorParams = struct;
[sensorParams.boundingbox.cornerx, sensorParams.boundingbox.cornery]... 
 = sensor_Bounds(readings);

n = 1;

seen = false;
for i = 1  : len/16 - 1
 sensorParams.sensorObj.UserData.scan = struct("distance", datum(16*(i-1) + 1 : 16*i, 1), "angle", datum(16*(i-1) + 1: 16*i, 2));
 measurement = filter_scan(sensorParams);
 if measurement.count > 0
  if ~seen
   firstUpdate = i;
   seen = true;
  end
  time =  timestep*16*(i - firstUpdate);

  [estimate, estimatorParams] = state_estimator([measurement.distance; measurement.angle], time, estimatorParams);
  collisionEstimate = collision_prediction(estimate, estimatorParams, collisionEstimate);
  est_vec(n) = estimate;
  col_vec(n) = collisionEstimate;
  m(n) = measurement;
  n = n + 1;
 else
   estimate.predState = nan(4,1);
   estimate.Ppred = nan(4,4);
   estimate.corrState = nan(4,1);
   estimate.Pcorr = nan(4,4);
 end
end

for j = 1 : length(est_vec)
 d(j,:) = est_vec(j).corrState(1:2);
 sigs = diag(est_vec(j).Pcorr);
 sigsUp(j,:) = d(j,:) + 2*sigs(1:2)'.^0.5;
 sigsDo(j,:) = d(j,:) - 2*sigs(1:2)'.^0.5;
 r(j) = m(j).distance*1000;
 th(j) = m(j).angle;
end
scatter(datum(:,1).*cosd(datum(:,2)), datum(:,1).*sind(datum(:,2)));
hold on
scatter(sensorParams.boundingbox.cornerx, sensorParams.boundingbox.cornery)
scatter(r.*cos(-th), r.*sin(-th))
figure
plot(d(:,1), d(:,2), sigsUp(:,1), sigsUp(:,2), sigsDo(:,1), sigsDo(:,2))
