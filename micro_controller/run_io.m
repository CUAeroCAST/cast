% This function handles serial output of the maneuver to the microcontroller.
function recv = run_io(maneuver, delay)
 fprintf('run_io is not implemented\n')
 send_delay = delay;
 send_maneuver = maneuver;
 recv = dummy_run_io(maneuver, delay, 1, 0);
end