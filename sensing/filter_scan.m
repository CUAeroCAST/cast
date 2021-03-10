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
 x = data.distance .* cosd(data.angle);
 y = data.distance .* sind(data.angle);
 inds = inpolygon(x, y, cornerx, cornery);
 inds = inds & data.distance~=0;
 count = sum(inds);
 objMeasurement.distance = data.distance(inds);
 objMeasurement.angle = data.angle(inds);


% Taking the mean of two object measurements in one scan
if count > 0
           
       objMeasurement.angle = deg2rad(meanangle(-objMeasurement.angle));
       objMeasurement.distance = mean(objMeasurement.distance)/1000;
       objMeasurement.count = count;
       
end
       
end
