function [position,t,state] = find_maneuver_position(satelliteState,burnTime,direction,timeToCol)
% Determines the satellite position after the maneuver
% Calls the ode function to integrate the acceleration of the spacecraft
% to get the state after the burn
% Inputs:
% satellietState: state and covarience of the satellite
% burnTime: time of the burn
% direction: direction of the burn
% timeToCol: time until the collision
% Outputs:
% position: satellite position at the collision time after the burn
tspan = 0:.001:timeToCol;
mu = 398600;
[t,state] =  ode45(@(t, state) orbit_prop_maneuver(t,state,mu,direction,burnTime),...
    tspan,satelliteState);
position = state(end,1:3);
end