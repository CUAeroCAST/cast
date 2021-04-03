function params = make_guidance_params()
 params.threshold = 0.50; % collision probability to trigger a maneuver
 params.rSensor = 0.08; % size of sensor integration box
 params.rBall = 0.025;
 params.rDodge = params.rSensor + params.rBall;
 params.discSize = 1e-2; % discretization size of gradient
 params.chiefState = [0;0;7578;7.25256299066873;0;0];
 params.scaling = 222;
 params.firstDetectionTime = 30; %seconds
 params.deputyState = [237.3344; -216.6338; 7.513e3; -0.6566; 7.2191; 0.2271];
end
