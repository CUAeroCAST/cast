function omegaDot = angular_accel(c1, c2, stepSize, frequency)
% calculates the actual angular acceleration experienced by a difference in
% step delays.
 omegaDot = 2 * stepSize * frequency^2 * (c1 - c2) / (c1 * c2 * (c1 + c2));
end