function objMeasurement = filter_scan(sensorParams)
 % filter scan data with bounding box to output a single object measurement
 
% Get data and bounding lines from sensor parameters
data = sensorParams.sensorObj.UserData.scan;
cornerx = sensorParams.boundingbox.cornerx;
cornery = sensorParams.boundingbox.cornery;
 
% Filter out object data
 objMeasurement = struct;
 objMeasurement.distance = [];
 objMeasurement.angle = [];
 objMeasurement.count = 0;
 for i = 1 : length(data)
  x = data(i).distance .* cosd(data(i).angle);
  y = data(i).distance .* sind(data(i).angle);
  inds = inpolygon(x, y, cornerx, cornery);
  inds = inds & data(i).distance~=0;
  count = sum(inds);
  objMeasurement.distance = [objMeasurement.distance, data(i).distance(inds)];
  objMeasurement.angle = [objMeasurement.angle, data(i).angle(inds)];
 end


% Taking the mean of two object measurements in one scan
if count > 0
           
       objMeasurement.angle = deg2rad(meanangle(objMeasurement.angle));
       objMeasurement.distance = mean(objMeasurement.distance)/1000;
       objMeasurement.count = count;
       
end
       
end
