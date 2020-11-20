function out = lam_vals(R)
% Takes the measurement noise covariance in polar coordinates and
% calculates lamda values necessary for bias removal
% Out is in the form [lam_beta, lam_beta']
out = [exp(-R(2,2));
       exp(-2*R(2,2))
       ];
end