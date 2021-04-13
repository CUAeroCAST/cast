function [xp,Pp,nis,diffs] = correct_ekf(filter,sensorReading, estimate, xs, ys)
%This function corrects the predicition based on received reading
%xs and ys are the sensor x and y locations
%xm is x,y,vx,vy
xm = estimate.predState;
Pm = estimate.Ppred;
%Measurement model
H = filter.MeasurementModel(xm(1), xs, xm(2), ys);
%Sensor Covariance
R = filter.MeasurementNoise;
%Kalman Filter Gain
K = Pm*(H')*inv(H*Pm*(H') + R);%Consider replacing with backslash
%Measurement estimate
y_hat = [sqrt((xm(1)-xs)^2+(xm(2)-ys)^2);
         atan2((xm(2)-ys),(xm(1)-xs))];
%Correction algo
diffs = sensorReading - y_hat;
xp = xm + K*(diffs)+[0.01;0;0.01;0];
Pp = (eye(4) - K*H)*Pm*(eye(4)-K*H)' + K*R*K';
S = H*Pm*H'+R;
nis = diffs'*inv(S)*diffs;
end
