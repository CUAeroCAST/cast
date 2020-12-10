function stepDelay = init_step(accel, stepSize, frequency)
 % calculates the initial step delay for a given acceleration,
 % motor, and clock speed
 stepDelay = round(frequency * sqrt(2 * stepSize / accel));
end