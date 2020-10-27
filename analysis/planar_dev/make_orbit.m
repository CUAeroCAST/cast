function ydot = make_orbit(t,y,mu,Omega_mag)
% generates a reference circulay orbit using 2 body acceleration
r = y(1:3);
v = y(4:6);


vdot = -Omega_mag^2*r;
rdot = v;
ydot = [rdot;vdot];