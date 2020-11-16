function dx = statetransitionfcn(x, dt, m)
% State transition function for the EKF. This is set up for polar
% coordinate range/bearing tracking following the derivation in the MIT
% paper. 
% x = [theta, R, Vx, Vy]^T
% m = [Vsx, Vsy, asx, asy]^T contains velocity and acceleration information
% for the observer
% dx = f(x); State transition model
% dL = [deltaLn, deltaLp] at time k
% rT and rS are the xy position of the target and observer respectively

s = [cos(x(1)), -sin(x(1));
     sin(x(1)), cos(x(1))];
rT = dt.*[x(3); x(4)];
rS = dt.*[m(1);m(2)] + (0.5*dt^2).*[m(3);m(4)];
dL = s*(rT - rS);

f = [x(1) + atan(dL(1)/(x(2)+dL(2)));
     sqrt((x(2)+dL(2))^2 + dL(1)^2);
     x(3);
     x(4)];
dx = f;
end