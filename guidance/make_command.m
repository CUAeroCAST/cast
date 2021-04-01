function [xpoly, ypoly] = make_command(maneuver, timeToCollision, guidanceParams)
 global burnTable chiefOrbit timeTable;
 direction = round(atand(maneuver(2) / maneuver(1)));
 if direction < 0
  direction = direction + 360;
 end
 burnState = burnTable{maneuver(3), direction};
 stateTime = timeTable{maneuver(3), direction};
 burnState2d = convert_2d(chiefOrbit, burnState(:, 1:6), guidanceParams);
 vels2d = burnState2d(:, 3:4) * maneuver(3) / timeToCollision;
 stateTime = stateTime * timeToCollision / maneuver(3);
 velDiffs = diff(vels2d,1);
 maneuverEnd = max(find(abs(velDiffs(:, 1)) <= 1e-5, 1), find(abs(velDiffs(:, 2)) <= 1e-5, 1));
 xpoly = polyfit(stateTime(1:maneuverEnd), vels2d(1:maneuverEnd, 1), 1);
 ypoly = polyfit(stateTime(1:maneuverEnd), vels2d(1:maneuverEnd, 2), 1);
 if abs(xpoly(1)) > 3
  xpoly(1) = sign(xpoly(1)) * 2;
 end
 if abs(ypoly(1)) > 3
  ypoly(1) = sign(ypoly(1)) * 2;
 end
end
