function out = conv_meas_bias(lam,y)
% Computes the measurement bias, mu, given lam and measurement
out = [y(1)*cos(y(2))*(lam(1)-1);
       y(1)*sin(y(2))*(lam(1)-1)];
end