function objMeasurement = filter_scan(sensorParams)
 % filter scan data with bounding box to output a single object measurement
 
% Get data and bounding lines from sensor parameters
data = sensorParams.sensorObj.UserData.scan;
r12 = sensorParams.boundingbox.r12;
r13 = sensorParams.boundingbox.r13;
r24 = sensorParams.boundingbox.r24;
r43 = sensorParams.boundingbox.r43;
 
% Filter out object data
 objMeasurement = struct;
 objMeasurement.distance = [];
 objMeasurement.angle = [];
 objMeasurement.count = 0;
 count = 1;
for i = 1:length(data.distance)
    
    theta = data.angle(i);
    rvec = [r12(theta);r13(theta);r24(theta);r43(theta)];
    rvec = rvec(rvec>0);
    
   if all(data.distance(i)<rvec)
       
       objMeasurement.distance(count) = data.distance(i);
       objMeasurement.angle(count) = data.angle(i);
       
       count = count+1;
       
   end
    
end

% Taking the mean of two object measurements in one scan
if count > 1
           
       %objMeasurement.time = mean(data.time);
       %objMeasurement.start = mean(data.start);
       %objMeasurement.qual = mean(data.qual);
       objMeasurement.angle = mean(objMeasurement.angle);
       objMeasurement.distance = mean(objMeasurement.distance);
       objMeasurement.count = count - 1;
       
end
       
end
