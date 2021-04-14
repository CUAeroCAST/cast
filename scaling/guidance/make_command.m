function [xpoly, ypoly] = make_command(maneuver, timeToCollision, guidanceParams)
 global burnTable chiefOrbit timeTable;
 colPrimary = guidanceParams.deputyState(1:3)' - chiefOrbit(1, 1:3);
 colPrimary = colPrimary / norm(colPrimary);
 directionShift = sign(colPrimary(2)) * dot(colPrimary, [1, 0, 0]);
 direction = round(atan2d(maneuver(2), maneuver(1)) + directionShift);
 if direction < 0
  direction = direction + 360;
 elseif direction > 360
  direction = direction - 360;
 end
 burnState = burnTable{maneuver(3), direction};
 stateTime = timeTable{maneuver(3), direction};
 burnVels2d = convert_2d(chiefOrbit, burnState(:, 1:6), guidanceParams);
 vels2d = burnVels2d * guidanceParams.firstDetectionTime / timeToCollision;
 stateTime = stateTime * timeToCollision / maneuver(3);
 velDiffs = diff(vels2d,1);
 maneuverEnd = max(find(abs(velDiffs(:, 1)) <= 1e-5, 1), find(abs(velDiffs(:, 2)) <= 1e-5, 1));
 xpoly = polyfit(stateTime(1:maneuverEnd), vels2d(1:maneuverEnd, 1), 1);
 ypoly = polyfit(stateTime(1:maneuverEnd), vels2d(1:maneuverEnd, 2), 1);
end
