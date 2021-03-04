function objMeasurement = filter_scan(sensorParams)
 % filter scan data with bounding box to output a single object measurement
 
% Get data and bounding lines from sensor parameters
Data = sensorParams.sensorObj.UserData.scan;
[r12, r13, r24, r43] = sensorParams.boundingbox;
 
data.time = Data(:,1);
data.start = Data(:,2);
data.qual = Data(:,3);
data.angle = Data(:,4);
data.distance = Data(:,5);
 
% Filter out object data
 count = 1;
for i = 1:length(data.distance)
    
    theta = data.angle(i);
    rvec = [r12(theta);r13(theta);r24(theta);r43(theta)];
    rvec = rvec(rvec>0);
    
   if all(data.distance(i)<rvec)
       
       objMeasurement.time(count) = data.time(i);
       objMeasurement.start(count) = data.start(i);
       objMeasurement.qual(count) = data.qual(i);
       objMeasurement.angle(count) = data.angle(i);
       objMeasurement.distance(count) = data.distance(i);
       
       count = count+1;
       
   end
    
end

% Taking the mean of two object measurements in one scan
if length(objMeasurement.distance)>1
           
       objMeasurement.time = mean(data.time);
       objMeasurement.start = mean(data.start);
       objMeasurement.qual = mean(data.qual);
       objMeasurement.angle(count) = mean(data.angle);
       objMeasurement.distance(count) = mean(data.distance(i));
       
end
       
end
