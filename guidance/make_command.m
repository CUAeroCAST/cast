function [xpoly, ypoly] = make_command(maneuver, timeToCollision)
 global burnTable chiefOrbit timeTable;
 direction = round(atand(maneuver(2) / maneuver(1)));
 burnState = burnTable{maneuver(3), direction};
 stateTime = timeTable{maneuver(3), direction};
 burnState2d = convert_2d(chiefOrbit, burnState(:, 1:6));
 vels2d = burnState2d(:, 3:4) * maneuver(3) / timeToCollision;
 stateTime = stateTime * timeToCollision / maneuver(3);
 xpoly = polyfit(stateTime, vels2d, 1);
 ypoly = polyfit(stateTime, vels2d, 1);
end
