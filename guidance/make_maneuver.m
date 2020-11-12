% This function implements a guidance algorithm to generate a maneuver and timing. This
% function MUST be rate limited such that the estimator operates in real time.
function [maneuver, delay] = make_maneuver(estimate, guidanceParams)
 fprintf('make_maneuver is not implemented\n')
 doSomething = {estimate, guidanceParams};
 maneuver = rand(2,1)*.1-.05;
 delay = 0;
end