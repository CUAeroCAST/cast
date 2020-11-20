function R_conv = get_conv_cov(R, lam, y)
% This function takes the polar sensor covariance, lambda values, and
% sensor measurement and returns the rela debiased converted measurement
% error
alpha_x = sin(y(2))^2 * sinh(R(2,2)) + cos(y(2))^2 * cosh(R(2,2));
alpha_y = sin(y(2))^2 * cosh(R(2,2)) + cos(y(2))^2 * sinh(R(2,2));
alpha_xy = 1; %Must be 1 for 2D 

Rxx = (y(1)^2*(alpha_x*alpha_xy - cos(y(2))^2) + R(1,1)*alpha_x*alpha_xy)*lam(1)^2;
% Rxx = -exp(-R(2,2))*(y(1)^2)*(cos(y(2))^2)+0.5*(y(1)^2 + R(1,1)^2)*(1+cos(2*y(2))*exp(-2*R(2,2)));
Rxy = (y(1)^2*(alpha_xy - lam(1)^(-2)) + R(1,1)*alpha_xy)*sin(y(2))*cos(y(2))*lam(2);
Ryx = -exp(-R(2,2))*(y(1)^2)*cos(y(2))*sin(y(2)) + 0.5*(y(1)^2 + R(1,1)^2)*sin(2*y(2))*exp(-2*R(2,2));
Ryy = (y(1)^2*(alpha_y*alpha_xy - sin(y(2))^2) + R(1,1)*alpha_y*alpha_xy)*lam(1)^2; %should this have alpha_y?

R_conv = [Rxx, Rxy; 
          Rxy, Ryy];
end