clear
clc
close all
importCast;
SPD = 24 * 3600;

filename = "livedata--1-5m--267mm-offset--bigmiss--2ndbulb--march8.txt";
writematrix(["time(s)", "distance(mm)", "angle(rad)"], filename);

sensorParams = struct;
sensorParams.scanMode = "express";
sensorParams.readsize = 84;
sensorParams.portstr = "/dev/tty.usbserial-0001";
sensorParams.sensorObj = serial_Sensor(sensorParams);
cleanup = onCleanup(@()clean_up(sensorParams.sensorObj));

scan_Request(sensorParams);

estimatorParams = struct;
estimatorParams.currentTime = now * SPD;
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

first_scan = struct;
first_scan.distance = [];
first_scan.angle = [];

while length(first_scan.distance) < 800
 pause(1e-6)
 if sensorParams.sensorObj.UserData.dataReady
  first_scan.distance = [first_scan.distance; sensorParams.sensorObj.UserData.scan.distance'];
  first_scan.angle = [first_scan.angle; sensorParams.sensorObj.UserData.scan.angle'];
  sensorParams.sensorObj.UserData.dataReady = false;
 end
end
fprintf("Setup Complete")

[sensorParams.boundingbox.cornerx, sensorParams.boundingbox.cornery]... 
 = sensor_Bounds(first_scan);

ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'Stop loop', ...
                         'Callback', 'delete(gcbf)');

n = 1;
while true
 pause(1e-6)
 if ~ishandle(ButtonHandle)
     break;
 end
 if sensorParams.sensorObj.UserData.dataReady
  measurement = filter_scan(sensorParams);
  time = now * SPD;
  sensorParams.sensorObj.UserData.dataReady = false;
  
  if measurement.count > 0
   [estimate, estimatorParams] = state_estimator([measurement.distance; measurement.angle], time, estimatorParams);
   collisionEstimate = collision_prediction(estimate, estimatorParams, collisionEstimate);
   est_vec(n) = estimate;
   col_vec(n) = collisionEstimate;
   m(n) = measurement;
   t(n) = time;
   n = n + 1;
  else
    estimate.predState = nan(4,1);
    estimate.Ppred = nan(4,4);
    estimate.corrState = nan(4,1);
    estimate.Pcorr = nan(4,4);
  end
 end
end

clean_up(sensorParams.sensorObj);
for j = 1 : length(est_vec)
 d(j,:) = est_vec(j).corrState(1:2);
 sigs = diag(est_vec(j).Pcorr);
 sigsUp(j,:) = d(j,:) + 2*sigs(1:2)'.^0.5;
 sigsDo(j,:) = d(j,:) - 2*sigs(1:2)'.^0.5;
 r(j) = m(j).distance*1000;
 th(j) = m(j).angle;
end
scatter(sensorParams.boundingbox.cornerx, sensorParams.boundingbox.cornery)
hold on
scatter(r.*cos(th), r.*sin(th))
scatter(first_scan.distance.*cosd(first_scan.angle), first_scan.distance.*sind(first_scan.angle))
figure
plot(d(:,1), d(:,2), sigsUp(:,1), sigsUp(:,2), sigsDo(:,1), sigsDo(:,2))
datum = [t',r',th'];
writematrix(datum, filename, "WriteMode", "append")



function clean_up(sensorObj)
 stop_Motor(sensorObj);
 delete(sensorObj);
end
