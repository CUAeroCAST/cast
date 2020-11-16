function F = statejacobianfcn(x, dt, m)
% State jacobian function for the EKF. This is set up for polar
% coordinate range/bearing tracking following the derivation in the MIT
% paper. 
% x = [theta, R, Vx, Vy]^T
% m = [Vsx, Vsy, asx, asy]^T contains velocity and acceleration information
% for the observer
% dx = f*x; State transition model
% dL = [deltaLn, deltaLp] at time k
% rT and rS are the del xy position of the target and observer respectively
% dLx and dLy are defined by eqn 3.35 and 3.36; 

s = [cos(x(1)), -sin(x(1));
     sin(x(1)), cos(x(1))];
rT = dt.*[x(3); x(4)];
rS = dt.*[m(1);m(2)] + (0.5*dt^2).*[m(3);m(4)];
dL = s*(rT - rS);

dLx = x(2)*sin(x(1)) + rT(1) - rS(1);
dLy = x(2)*cos(x(1)) + rT(2) - rS(2);

F = [x(2)*(x(2) + dL(2))/(x(2)^2), dL(1)/(x(2)^2), dt*dLy/(x(2)^2), -(dt*dLx/(x(2)^2));
     x(2)*dL(1)/x(2), (x(2) + dL(2))/x(2), dt*dLx/x(2), dt*dLy/x(2);
     0,0,1,0;
     0,0,0,1];
end