function [x,P] = predict_kf(filter,dt)
%Predicts the state and covariance at time t + dt, without updating the
%time stored in the filter object

%F is the stm for DT
F = filter.STM(dt);
%Q is the process noise matrix
Q = filter.ProcessNoise(dt);

%Prediction algo
x = F*filter.State;
P = F*filter.StateCovariance*(F') + Q;
end

