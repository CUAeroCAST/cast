clc;
clear;
close all

Data=importdata("data1.txt");
data_Full=Data.data;
data_Sort=sortrows(data_Full,4);
data.time=data_Full(:,1);
data.start=data_Full(:,2);
data.qual=data_Full(:,3);
data.angle=data_Full(:,4);
data.distance=data_Full(:,5);

%get x,y from distance/angle then:
%%
rangelow = 1;
rangehigh = length(data.distance);
x = data.distance(rangelow:rangehigh) .* cosd(data.angle(rangelow:rangehigh));
y = data.distance(rangelow:rangehigh) .* sind(data.angle(rangelow:rangehigh));

% shortside = 1524;
shortside = 1112.52; % 43.8 in
tolerance = 50;

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
figure
hold on
scatter(x,y)
scatter([corner1(1), corner2(1)], [corner1(2),corner2(2)], "LineWidth", 10)
%%

delta_corner1=corner2-corner1; % From corner 1 to corner 2
delta_corner1_unit=delta_corner1/norm(delta_corner1);
delta_corner2_unit=-delta_corner1_unit;


bigpos_rel=bigpos-corner1; 
bigpos_vec=bigpos_rel-dot(bigpos_rel,delta_corner1_unit)*delta_corner1_unit;
bigpos_unit=bigpos_vec/norm(bigpos_vec);

figure;
scatter([corner1(1), corner2(1), bigpos(1)],[corner1(2), corner2(2), bigpos(2)])
hold on
quiver([corner1(1),corner1(1),corner2(1)],[corner1(2),corner1(2),corner2(2)],[delta_corner1_unit(1),bigpos_unit(1),delta_corner2_unit(1)],[delta_corner1_unit(2),bigpos_unit(2),delta_corner2_unit(2)])

bound_step=100; %mm
len=(95.37-6.40651)*25.4;
len=(86.85-6.40651)*25.4; % mm (long gantry distance-distance ramp comes into gantry) 86.85

corner1_bound=corner1+bound_step*(delta_corner1_unit+bigpos_unit);
corner2_bound=corner2+bound_step*(delta_corner2_unit+bigpos_unit);

corner3_bound=corner1_bound+(len-bound_step)*(bigpos_unit);% + 5*bound_step*delta_corner1_unit;
corner4_bound=corner2_bound+(len-bound_step)*(bigpos_unit);% + 5*bound_step*delta_corner2_unit;

m12=(corner1_bound(2)-corner2_bound(2))/(corner1_bound(1)-corner2_bound(1));
b12=corner1_bound(2)-m12*corner1_bound(1);

m13=(corner1_bound(2)-corner3_bound(2))/(corner1_bound(1)-corner3_bound(1));
b13=corner1_bound(2)-m13*corner1_bound(1);

m24=(corner2_bound(2)-corner4_bound(2))/(corner2_bound(1)-corner4_bound(1));
b24=corner2_bound(2)-m24*corner2_bound(1);

m43=(corner4_bound(2)-corner3_bound(2))/(corner4_bound(1)-corner3_bound(1));
b43=corner4_bound(2)-m43*corner4_bound(1);


r12= @(theta) b12./(sind(theta)-m12*cosd(theta));
r13= @(theta) b13./(sind(theta)-m13*cosd(theta));
r24= @(theta) b24./(sind(theta)-m24*cosd(theta));
r43= @(theta) b43./(sind(theta)-m43*cosd(theta));

thetatest=0:365;
figure;
polarplot(thetatest*pi/180,r12(thetatest)), hold on
polarplot(thetatest*pi/180,r13(thetatest)), hold on
polarplot(thetatest*pi/180,r24(thetatest)), hold on
polarplot(thetatest*pi/180,r43(thetatest)), hold on
rlim([0 3000])

count=1;
for i=1:length(data.distance)
    
    theta=data.angle(i);
    rvec=[r12(theta);r13(theta);r24(theta);r43(theta)];
    rvec=rvec(rvec>0);
    
   if all(data.distance(i)<rvec)
       
       obj_Data.time(count)=data.time(i);
       obj_Data.start(count)=data.start(i);
       obj_Data.qual(count)=data.qual(i);
       obj_Data.angle(count)=data.angle(i);
       obj_Data.distance(count)=data.distance(i);
       
       count=count+1;
       
   end
    
end


figure;
scatter(obj_Data.distance .*cosd(obj_Data.angle), obj_Data.distance .*sind(obj_Data.angle),'LineWidth',3), hold on

mark_5Sec=find(data.time==5000);
data_Sort=sortrows(data_Full(1:mark_5Sec,:),4);

figure;
for i=0:360
    ind=find(data_Sort(:,4)==i);
    ind_Count(i+1)=length(ind);
    mean_Dist(i+1)=sum(data_Sort(ind,5))/ind_Count(i+1);
    scatter(data_Sort(ind,5).*cosd(data_Sort(ind,4)), data_Sort(ind,5).*sind(data_Sort(ind,4)),'LineWidth',3), hold on
end

data_Sort=sortrows(data_Full(mark_5Sec+1:end,:),4);
% for i=0:365
%     ind=find(data_Sort(:,4)==i);
%     ind_Count(i+1)=length(ind);
%     mean_Dist(i+1)=sum(data_Sort(ind,5))/ind_Count(i+1);
% end
count=1;
for i=1:length(data_Sort)
    angle=data_Sort(i,4);
    
        if abs(data_Sort(i, 5)-mean_Dist(angle+1))>=152.4 && abs(data_Sort(i, 5)-mean_Dist(angle+1))<=1016
        
        obj_Data.time(count)=data_Sort(i, 1);
        obj_Data.start(count)=data_Sort(i, 2);
        obj_Data.qual(count)=data_Sort(i, 3);
        obj_Data.angle(count)=data_Sort(i, 4);
        obj_Data.distance(count)=data_Sort(i, 5);
        
        count=count+1;
        
        end
    
end
% figure;
% ang=0:365;
% scatter(mean_Dist.*cosd(ang), mean_Dist.*sind(ang),'LineWidth',3), hold on

figure;
scatter(data.distance.*cosd(data.angle), data.distance.*sind(data.angle),'LineWidth',3), hold on
title("LiDAR Sensor Surroundings Plot",'fontsize',36,'Interpreter','latex')
xlabel('x [mm]','fontsize',36,'Interpreter','latex')
ylabel('y [mm]','FontSize',36,'Interpreter','latex')
%set(gca, 'FontSize', 36)
grid minor;



figure;
scatter(obj_Data.distance.*cosd(obj_Data.angle), obj_Data.distance.*sind(obj_Data.angle),'LineWidth',3), hold on
title("LiDAR Sensor Surroundings Plot",'fontsize',24,'Interpreter','latex')
xlabel('x [mm]','fontsize',22,'Interpreter','latex')
ylabel('y [mm]','FontSize',24,'Interpreter','latex')
set(gca, 'FontSize', 20)
grid minor;


r12= @(theta) (b12)./(sind(theta)-m12*cosd(theta));
r13= @(theta) (b13)./(sind(theta)-m13*cosd(theta));
r24= @(theta) (b24)./(sind(theta)-m24*cosd(theta));
r43= @(theta) (b43)./(sind(theta)-m43*cosd(theta));

count=1;
for i=1:length(data.distance)
    
    theta=data.angle(i);
    rvec=[r12(theta);r13(theta);r24(theta);r43(theta)];
    rvec=rvec(rvec>0);
    
   if all(data.distance(i)<rvec)
       
       obj_Data.time(count)=data.time(i);
       obj_Data.start(count)=data.start(i);
       obj_Data.qual(count)=data.qual(i);
       obj_Data.angle(count)=data.angle(i);
       obj_Data.distance(count)=data.distance(i);
       
       count=count+1;
       
   end
    
end

% NOTE: The below code for rotating the plot was used from: https://www.mathworks.com/matlabcentral/answers/93554-how-can-i-rotate-a-set-of-points-in-a-plane-by-a-certain-angle-about-an-arbitrary-point
x = data.distance.*cosd(data.angle);
x = x';
y = data.distance.*sind(data.angle);
y = y';
v = [x;y];
x_center = 0;
y_center = 0;
center = repmat([x_center; y_center], 1, length(x));
theta = (pi/2)+(pi/4)+(pi);
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
s = v - center;     % shift points in the plane so that the center of rotation is at the origin
so = R*s;           % apply the rotation about the origin
vo = so + center;   % shift again so the origin goes back to the desired center of rotation
x_rotated = vo(1,:);
y_rotated = vo(2,:);
% NOTE: The above code for rotating the plot was used from: https://www.mathworks.com/matlabcentral/answers/93554-how-can-i-rotate-a-set-of-points-in-a-plane-by-a-certain-angle-about-an-arbitrary-point
figure;

figure;
hold on
%scatter(obj_Data.distance.*cosd(obj_Data.angle), obj_Data.distance.*sind(obj_Data.angle),'LineWidth',3);
scatter(x_rotated, y_rotated,'r','LineWidth',3);
scatter(0, 0,'xk','LineWidth',3);
%camroll(90+45)
 %rotate(h,[0 0],90+45)
 %rotate(g,[0 0],90+45)
ylim([-3500,1000])
title("LiDAR Sensor Surroundings Plot",'fontsize',36,'Interpreter','latex')
xlabel('x [mm]','fontsize',36,'Interpreter','latex')
ylabel('y [mm]','FontSize',36,'Interpreter','latex')
%view([45 45])
%set(gca, 'FontSize', 36)
set(gca,'YDir','reverse');
grid minor;


% figure
% i=1;
% for n=1:length(data.distance) % use for loop to animate the line
%         scatter(double(data.distance(n)).*cosd(double(data.angle(n))), double(data.distance(n)).*sind(double(data.angle(n))),'LineWidth',3), hold on
%         i=i+1;
%         grid on
%         %drawnow
%         pause(0.00005)
% end
