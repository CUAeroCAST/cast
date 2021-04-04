function data = generate_test_data(sensorParams, targetParams,...
    initSpeed, rampPos, rampAng)
vx = -initSpeed * cos(rampAng);%Negative to account for ramp shooting towards 0,0
vy = initSpeed * sin(rampAng);
t_end = (0-rampPos(1))/vx; %x-position relative to sensor/velocity in x
%floor to handle cases where t_end is repeating
n_samples = floor(t_end*sensorParams.samplingRate);
timeVec = linspace(0,t_end, n_samples)';
dt = timeVec(2)-timeVec(1);
% Noise Stuff
Q = 0.05*eye(2);
Omega = [dt^2/2 0;0 dt^2/2;dt 0;0 dt];
F = [1 0 dt 0;0 1 0 dt;0 0 1 0;0 0 0 1];
%Constant velocity kinematics
% x_rel = linspace(rampPos(1), rampPos(1) + vx*t_end, n_samples)';
% y_rel = linspace(rampPos(2), rampPos(2) + vy*t_end, n_samples)';
z_rel = zeros(n_samples,1);
state = zeros(4,n_samples);
state(:,1) = [rampPos(1);rampPos(2);vx;vy];
for i = 2:n_samples
    v = mvnrnd(zeros(2,1),Q)';
    state(:,i) = F*state(:,i-1)+Omega*v;
end
x_rel = state(1,:)';
y_rel = state(2,:)';
%relative path is xyz
relativePath = [x_rel, y_rel, z_rel];
sensorScenario = init_sensor_model(relativePath, timeVec, sensorParams,...
                               targetParams);
makeSensorPlot = 0; %false
sensorReadings = sensor_model(sensorScenario, makeSensorPlot);
data = [timeVec, sensorReadings(:,1), rad2deg(wrapTo2Pi(sensorReadings(:,2)))];
%remove NaNs
data(any(isnan(data),2),:) = [];
%remove ranges below minimum range
data(data(:,2) < 0.15, :) = [];
end