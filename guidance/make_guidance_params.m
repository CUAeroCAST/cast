function params = make_guidance_params()
 params.threshold = 0.50; % collision probability to trigger a maneuver
 params.rSensor = 0.08; % size of sensor integration box
 params.discSize = 1e-2; % discretization size of gradient
 params.chiefState = [0;0;7578;7.25256299066873;0;0];
 params.scaling = 222;
end
