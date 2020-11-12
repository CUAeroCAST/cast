function accel = init_accel(stepDelay, stepSize, frequency)
% calculates the initial acceleration for a given step delay, motor, and clock
 accel = stepSize * (frequency / stepDelay)^2;
end