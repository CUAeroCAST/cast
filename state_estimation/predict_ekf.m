function [x,P] = predict_ekf(filter,dt,t)
%Predicts the state and covariance at time t + dt, without updating the
%time stored in the filter object

%F is the stm for DT
F = filter.STM(dt);
%Q is the process noise matrix
Q = filter.ProcessNoise(t);

Omega = [dt^2/2 0; 0 dt^2/2; dt 0; 0 dt];

%Prediction algo
x = F*filter.State;
P = F*filter.StateCovariance*(F') + Omega*Q*Omega';
end

