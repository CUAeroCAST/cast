function params = make_estimator_params(noiseNum)
 SPD = 24 * 3600;
 params.currentTime = now * SPD;
 params.filter.MeasurementModel = @(x,xs,y,ys)...
     [(2*x - 2*xs)/(2*((x - xs)^2 + (y - ys)^2)^(1/2)),...
     (2*y - 2*ys)/(2*((x - xs)^2 + (y - ys)^2)^(1/2)), 0, 0;
     -(y - ys)/((x - xs)^2*((y - ys)^2/(x - xs)^2 + 1)),...
     1/((x - xs)*((y - ys)^2/(x - xs)^2 + 1)), 0, 0]; %Range bearing model
 params.filter.MeasurementNoise = [0.025^2,0;0,deg2rad(0.45)^2]; %Range, bearing
 params.filter.ProcessNoise = @(t)  diag([1e-7,1e-5,1e-3,5e-7]); %constant process noise
 params.filter.StateCovariance = diag([5e-2,1e-4,5e-4,1e-4]); %Initial estimate covariance
%  params.filter.ProcessNoise = @(t)  diag([1e-5,1e-4,1e-3,1e-5]); %constant process noise
%  params.filter.ProcessNoise = @(t)  diag([1e-5,1e-4]);
 params.filter.STM = @(dt) eye(4) + [0,0,1,0;0,0,0,1;0,0,0,0;0,0,0,0]*dt;
 params.filter.State = [1.5;0;-1;0];
 params.xs = 0;
 params.ys = 0;
end