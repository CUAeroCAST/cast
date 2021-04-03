clear
clc
close all
importCast;
SPD = 24 * 3600;

global datapath;
filename = "nothing.txt";

datapath = open_logging(true);

writematrix(["time(s)", "distance(mm)", "angle(rad)"], filename);

sensorParams = make_sensor_params();
sensorParams = build_bounding_box(sensorParams);
estimatorParams = make_estimator_params();


fprintf("Setup Complete")

ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'Stop loop', ...
                         'Callback', 'delete(gcbf)');
collisionEstimate(1) = struct("collisionTime", nan, "predState", nan, "Ppred", nan);                        

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

log_struct(est_vec, [datapath, filesep, 'estimates']);
log_struct(m, [datapath,filesep, 'measurements'])

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
scatter(first_scan.distance.*cosd(-first_scan.angle), first_scan.distance.*sind(-first_scan.angle))
figure
plot(d(:,1), d(:,2), sigsUp(:,1), sigsUp(:,2), sigsDo(:,1), sigsDo(:,2))
datum = [t',r',th'];
writematrix(datum, filename, "WriteMode", "append")



function clean_up(sensorObj)
 stop_Motor(sensorObj);
 %delete(sensorObj);
end
