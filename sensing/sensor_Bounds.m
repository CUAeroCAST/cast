function [r12,r13,r24,r43] = sensor_Bounds(Data)

% Load in data
data.time=Data(:,1);
data.start=Data(:,2);
data.qual=Data(:,3);
data.angle=Data(:,4);
data.distance=Data(:,5);

% Get x,y from distance/angle
rangelow = 1;
rangehigh = length(data.distance);
x = data.distance(rangelow:rangehigh) .* cosd(data.angle(rangelow:rangehigh));
y = data.distance(rangelow:rangehigh) .* sind(data.angle(rangelow:rangehigh));

shortside = 1524;
tolerance = 50;

% Find first corners
pos=[x,y];
mags = vecnorm(pos,2,2);
ind = find(mags == max(mags));
bigpos = [x(ind), y(ind)];
deltapos = bigpos - pos;
mags = vecnorm(deltapos,2,2);
ind = find(mags == max(mags), 1);
corner1 = [x(ind), y(ind)];
rel_corner = corner1 - pos;
rel_corner_mag = vecnorm(rel_corner,2,2);
ind = find((rel_corner_mag > (shortside - tolerance)) & (rel_corner_mag < (shortside + tolerance)));
cornersmaybe = [x(ind), y(ind)];
deltapos = bigpos - cornersmaybe;
mags = vecnorm(deltapos,2,2);
ind = find(mags == max(mags),1);
corner2 = [cornersmaybe(ind,1), cornersmaybe(ind,2)];

% Finding direction of corner vectors
delta_corner1=corner2-corner1; % From corner 1 to corner 2
delta_corner1_unit=delta_corner1/norm(delta_corner1);
delta_corner2_unit=-delta_corner1_unit;

% Stepping into gantry limits (bound) by the bound_step distance
bigpos_rel=bigpos-corner1; 
bigpos_vec=bigpos_rel-dot(bigpos_rel,delta_corner1_unit)*delta_corner1_unit;
bigpos_unit=bigpos_vec/norm(bigpos_vec);

bound_step=100; %mm
len=(95.37-6.40651)*25.4; % mm (long gantry distance-distance ramp comes into gantry)

corner1_bound=corner1+bound_step*(delta_corner1_unit+bigpos_unit);
corner2_bound=corner2+bound_step*(delta_corner2_unit+bigpos_unit);

corner3_bound=corner1_bound+(len-bound_step)*(bigpos_unit);
corner4_bound=corner2_bound+(len-bound_step)*(bigpos_unit);

% Creating lines that define the bounds in polar
m12=(corner1_bound(2)-corner2_bound(2))/(corner1_bound(1)-corner2_bound(1));
b12=corner1_bound(2)-m12*corner1_bound(1);

m13=(corner1_bound(2)-corner3_bound(2))/(corner1_bound(1)-corner3_bound(1));
b13=corner1_bound(2)-m13*corner1_bound(1);

m24=(corner2_bound(2)-corner4_bound(2))/(corner2_bound(1)-corner4_bound(1));
b24=corner2_bound(2)-m24*corner2_bound(1);

m43=(corner4_bound(2)-corner3_bound(2))/(corner4_bound(1)-corner3_bound(1));
b43=corner4_bound(2)-m43*corner4_bound(1);

% Turning the polar lines into function handles
r12= @(theta) b12./(sind(theta)-m12*cosd(theta));
r13= @(theta) b13./(sind(theta)-m13*cosd(theta));
r24= @(theta) b24./(sind(theta)-m24*cosd(theta));
r43= @(theta) b43./(sind(theta)-m43*cosd(theta));

end