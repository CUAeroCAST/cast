% This function handles serial output of the maneuver to the microcontroller.
function recv = dummy_run_io(motorInput, delay, controlType, timeStep)
  if controlType == 1 %make motor match position
      recv.state = motorInput;
  else %make motors match acceleration
      %
      fprintf('Acceleration match is not implemented\n')
  end
end