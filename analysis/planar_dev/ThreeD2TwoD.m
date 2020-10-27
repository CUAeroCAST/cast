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
tprop = [0:.5:T];
numAnglesTested = 10;
Ri = R_orbit * [0;0;1];  %zdir = R_orbit
thetas = deg2rad(linspace(-135,-45,numAnglesTested)); %collision angles
subplot(1,2,1)

%plot 500 second interval for each collision angle
for i = 1:numAnglesTested
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
for i = 1:numAnglesTested
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





