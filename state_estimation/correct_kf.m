function [x,P] = correct_kf(filter,y, estimate)
%This function corrects the predicition based on received reading
xm = estimate.predState;
Pm = estimate.Ppred;
%Measurement model
H = filter.MeasurementModel;
%Sensor Covariance
R = filter.MeasurementNoise;
%Kalman Filter Gain
K = Pm*(H')*inv(H*Pm*(H') + R);
%Correction algo
x = xm + K*(y - H*xm);
P = (eye(4) - K*H)*Pm;
end