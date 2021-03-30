function params = make_estimator_params()
 SPD = 24 * 3600;
 params.currentTime = now * SPD;
 params.filter.MeasurementModel = @(x,xs,y,ys)...
     [(2*x - 2*xs)/(2*((x - xs)^2 + (y - ys)^2)^(1/2)),...
     (2*y - 2*ys)/(2*((x - xs)^2 + (y - ys)^2)^(1/2)), 0, 0;
     -(y - ys)/((x - xs)^2*((y - ys)^2/(x - xs)^2 + 1)),...
     1/((x - xs)*((y - ys)^2/(x - xs)^2 + 1)), 0, 0]; %Range bearing model
 params.filter.MeasurementNoise = [0.025^2,0;0,deg2rad(0.45)^2]; %Range, bearing
 params.filter.ProcessNoise = @(dt) 0.005*eye(4); %constant process noise
 params.filter.StateCovariance = eye(4); %Initial estimate covariance
 params.filter.STM = @(dt) eye(4) + [0,0,1,0;0,0,0,1;0,0,0,0;0,0,0,0]*dt;
 params.filter.State = [1.5;0;-1;0];
 params.xs = 0;
 params.ys = 0;
end
