% Stepper Control Feasibility Analysis
% reference: https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/
% author: Trace Valade
% project: CU-CAST
% date: Nov 11, 2020

%% HOUSEKEEPING
clear; clc; close all

%% USER CONFIG
% angle the motor can step
stepSize = deg2rad(1.8/4);
% clock frequency of micro controller
frequency = 16e6;
tmax = 2;
% target acceleration curve
% t = 0:0.1:10;
% omegaDots = cos(2*pi*t)+1;
load thrusterData
omegaDots = omegaDots/10;
%% INIT
% generate lookup table with interpolation
interpolant = griddedInterpolant(t, omegaDots);

% generate initial step delay and acceleration
stepDelay = init_step(omegaDots(1), stepSize, frequency);
avgOmegaDot = init_accel(stepDelay, stepSize, frequency);

% init loop variable and data storage structures
stepNum = 0;
tau = 0;

taus = [];
omegas = [];
thetas = [];

times = [];
accels = [];

delays = [];

%% MAIN LOOP
while (tau < t(end)) && (tau < tmax)
 % increment step number and get statistics from previous step
 stepNum = stepNum + 1;
 timeDelay = time_delay(stepDelay, frequency);
 omegai = angular_velocity(timeDelay, stepSize);
 omegaDot = interpolant(tau);
 delays = [delays, stepDelay];
 % get target acceleration and calculate step delay and actual acceleration
 omegaDotNext = interpolant(tau + timeDelay);
 stepDelayNext = step_delay(stepDelay, omegaDotNext, stepSize, frequency);
 avgOmegaDot = angular_accel(stepDelay, stepDelayNext, stepSize, frequency);
 
 % store data for processing
 taus = [taus, tau, tau + timeDelay];
 omegas = [omegas, omegai, omegai];
 thetas = [thetas, (stepNum-1) * stepSize, stepNum * stepSize];
 times = [times, ...
  tau + timeDelay/2, tau + timeDelay + time_delay(stepDelayNext, frequency)/2];
 accels = [accels, avgOmegaDot, avgOmegaDot];
 
 % increment timer and loop variables
 tau = tau + timeDelay;
 stepDelay = stepDelayNext; 
end

%% PLOTTING
figure
sgtitle('Open Loop Stepper Response')
subplot(1,3,1)
plot(t, omegaDots, times, accels)
hold on
grid minor
xlim([0,times(end)])
xlabel('Time (s)')
ylabel('Angular Acceleration')
legend('Target','Response')

subplot(1,3,2)
plot(taus, cumtrapz(taus, interpolant(taus)), taus, omegas)
hold on
grid minor
xlim([0,taus(end)])
xlabel('Time (s)')
ylabel('Angular Velocity')

subplot(1,3,3)
plot(taus, cumtrapz(taus, cumtrapz(taus, interpolant(taus))), taus, thetas)
hold on
grid minor
xlim([0,taus(end)])
xlabel('Time (s)')
ylabel('Angular Position')

figure
plot(delays)