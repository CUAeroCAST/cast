%% 3D trajectory projection on to 2D viewing plane of avoiding object
%   Purpose: Gain insight into deviation of trasnlating 3D collision
%   scenario onto a plane. What degree will the collision be rectilinear?
%   CAST - Griffin Van Anne
%   ODE orbit source code - Trace Valade
%   Date: 10/1/2020

%% Housekeeping
 clear
 close all
 clc

figure
sgtitle('Relative Position of Object 2 w/respect to 2D Observation Plane of Object 1')
mu =  3.986004*10^14/(10^(9)); %km^3/s^2
orbitingAlt = 1200;    %km
Rearth = 6371;        %km

R_orbit = (orbitingAlt+Rearth); %km
Vmag = sqrt(mu/R_orbit); %km/s 
Omega_mag = Vmag/R_orbit; %rad/s
T = 2*pi*sqrt(R_orbit^3/mu);   %orbital period - s
tprop = 0:.5:T;

Ri = R_orbit * [0;0;1];  %zdir = R_orbit
thetas = deg2rad(linspace(-135,-45,10)); %collision angles
subplot(1,2,1)

%plot 500 second interval for each collision angle
for i = 1:10
    phi = tan(thetas(i));
    x = sqrt(1/(phi^2+1));
    y = x * phi;
    if thetas(i)<-pi/2
       x = -x;
       y = -y;
    end
    Vi_obj1 = [1;0;0]*Vmag;%since z is at it's max value, must be restricted to x-y plane
    Vi_obj2 = [x;y;0]*Vmag;
    
    State1 = [Ri;Vi_obj1];
    State2 = [Ri;Vi_obj2];
    [error500(i),angles500(i,:)] = plotintersect(State1, State2,500);

end

%plot 100 second interval for each collision angle
subplot(1,2,2)
for i = 1:10
    phi = tan(thetas(i));
    x = sqrt(1/(phi^2+1));
    y = x * phi;
    if thetas(i)<-pi/2
       x = -x;
       y = -y;
    end
    Vi_obj1 = [1;0;0]*Vmag;%since z is at it's max value, must be restricted to x-y plane
    Vi_obj2 = [x;y;0]*Vmag;
    
    State1 = [Ri;Vi_obj1];
    State2 = [Ri;Vi_obj2];
    [error100(i),angles100(i,:)] = plotintersect(State1, State2,100);
    
end

function [error,dev_fromPlane] = plotintersect(State1, State2,timeplot)
% Purpose: Propagate orbits for 2 objects... convert inertial position to
% relative positions. Then project relative position onto the plane of the
% first object. 
%% Initial Parameters
mu =  3.986004*10^14/(10^(9)); %m^3/s^2
orbitingAlt = 800;    %km
Rearth = 6371;        %km

R_orbit = (orbitingAlt+Rearth); %km
Vmag = sqrt(mu/R_orbit); %km/s 
Omega_mag = Vmag/R_orbit; %rad/s
T = 2*pi*sqrt(R_orbit^3/mu);   %s
tprop = [0:.5:T];


%% Call to ODE
[times,orbit1] = ode45(@(t,y) make_orbit(t,y,mu,Omega_mag), tprop, State1);
[times,orbit2] = ode45(@(t,y) make_orbit(t,y,mu,Omega_mag), tprop, State2);


%% Projection
radius1 = orbit1(:,1:3);
normalVect = radius1/R_orbit; %normal vector defining plane is just the r unit vector
relPos = -orbit1(:,1:3) + orbit2(:,1:3); %object 2 w/r to object 1
proj2_plane1 = relPos(:,1:3) - dot(relPos(:,1:3),normalVect,2).*normalVect;% perfom the projection

%normal direction, since normal vect is a unit vector, this should be the
%deviation from plane in km
d_fromPlane = dot(relPos(:,1:3),normalVect,2);

%% Transformation matrices
% must transform object 1's plane in each timestep back to an xy plane so
% that relative positions for each time can be plotted
for i = 1:length(normalVect)
   X = normalVect(i,1)*orbit1(i,1); 
   Z = normalVect(i,3)*orbit1(i,3); 
   x0 = [orbit1(i,1) orbit1(i,2) orbit1(i,3)];
   deg = -pi/2+atan(normalVect(i,3)/normalVect(i,1)); %rotation degree for transformation
   k = [0;1;0];
   %[trans(i,:),M,t]=AxelRot(proj2_plane1(i,:)',deg,[0 1 0],x0);
   %trans(i,:) = ((M*(proj2_plane1(i,:)'))+t)';
   vect = proj2_plane1(i,:)';
   %vect = normalVect(i,:)';
   trans(i,:) = vect*cos(deg) + cross(k,vect)*sin(deg)+k*(dot(k,vect))*(1-cos(deg)); %relative positions tranformed to x,y,z coords... z = 0 which means transformation was successful 
   dev_fromPlane(i) = atand(d_fromPlane(i)/sqrt(trans(i,1)^2+trans(i,2)^2));% angular deviation is computed as tan^-1(n/r) where r is relative distance(in plane), n is distance in normal dir
end
dev_fromPlane = dev_fromPlane(end-timeplot*2:end);

%% Plotting

str = sprintf('Last %d seconds before collision',timeplot);
title(str)
xlabel('Relative x position of object 2(km)')
ylabel('Relative y position of object 2(km)')

y = -trans(end-timeplot*2,1)/trans(end-timeplot*2,2) * trans(end-timeplot*2:end,2);
hold on
%plot(trans(end-timeplot*2:end,2),y)
plot(-trans(end-timeplot*2:end,2),-trans(end-timeplot*2:end,1),'linewidth',2)

axis equal

%% Error of linearity
error = rsquare(-trans(end-timeplot*2:end,1),y);

end

%% functions
function ydot = make_orbit(t,y,mu,Omega_mag)
% generates a reference circulay orbit using 2 body acceleration
r = y(1:3);
v = y(4:6);


vdot = -Omega_mag^2*r;
rdot = v;
ydot = [rdot;vdot];
end