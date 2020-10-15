function [xt,yt,zt,ut,vt,wt] = CW(x0,y0,z0,xdot0,ydot0,zdot0,n,t)
 xt = (4*x0)+((2/n)*ydot0)+((xdot0/n)*sin(n*t))-((3*x0+2*ydot0/n)*cos(n*t));
 yt = y0-((2/n)*xdot0)-(3*(2*n*x0+ydot0)*t)+(2*(3*x0+2*ydot0/n)*sin(n*t))+(2/n)*xdot0*cos(n*t);
 zt = (1/n)*zdot0*sin(n*t)+z0*cos(n*t);
 ut = 3*n*sin(n*t)*x0+cos(n*t)*xdot0+2*sin(n*t)*ydot0;
 vt = 6*n*(cos(n*t)-1)*x0-2*sin(n*t)*xdot0+(4*cos(n*t)-3)*ydot0;
 wt = -n*sin(n*t)*z0+cos(n*t)*zdot0;
end