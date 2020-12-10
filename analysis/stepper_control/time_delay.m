function dt = time_delay(stepDelay, frequency)
% calculates the time delay from clock frequency and step delay
 dt = stepDelay / frequency;
end