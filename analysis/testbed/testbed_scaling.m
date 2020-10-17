close all

rel_vel = [7.003414400152234; 13.593872740250115]*1000;
time = 10;
distance = rel_vel * time;

sizes = 1:10:101;
bb_size = 0.005;
%distance = 135938;

% view_angle = size/distance
% orbit_view = size(something)/distance(collision event)
% ======================================
% testbed_view = size(bb) / distance(something)

view_angles = sizes./distance;
test_bed_distances = bb_size./view_angles;
test_bed_velocities = test_bed_distances/time;

figure
hold on
grid minor
title('Size and velocity for 1200km scenario by BB size target')
xlabel('BB size target (m)')
yyaxis left
ylabel('Testbed size (m)')
plot(sizes, test_bed_distances)
yyaxis right
ylabel('Testbed velocity (m/s)')
plot(sizes, test_bed_velocities)
legend('Crosstrack (7km/s)', 'Headon (13.6km/s)')