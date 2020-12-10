function stepDelay = step_delay(stepDelay, accel, stepSize, frequency)
% calculates the step delay from the previous step and the target acceleration
% solves the quadratic form below for delay2 and uses the upper value. Lower value is
% necessary when using negative accelerations, however imaginary results occur when
% accelerations are between (stepSize)(freq/delay1)^2(-6 \pm sqrt(5)). This function
% should only be used to track positive accelerations.
% omegaDot = \frac{2(stepSize)(freq^2)(delay1 -delay2)}{(delay1)(delay2)(delay1 + delay2)}
 b = (stepDelay + 2 * stepSize * frequency^2 / (accel * stepDelay));
 c = -2 * stepSize * frequency^2 / accel;
 stepDelay = round((-b + sqrt(b^2 - 4*c)) / 2);
end