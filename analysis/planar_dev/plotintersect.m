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