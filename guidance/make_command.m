function [xpoly, ypoly] = make_command(maneuver, timeToCollision, guidanceParams)
 global burnTable chiefOrbit timeTable;
 
 direction = round(atan2d(maneuver(2), maneuver(1)));
 if direction < 0
  direction = direction + 360;
 elseif direction > 360
  direction = direction - 360;
 end
 burnState = burnTable{maneuver(3), direction};
 stateTime = timeTable{maneuver(3), direction};
 % get 2D, convert back to sensor frame
 burnVels2d = convert_2d(chiefOrbit, burnState(:, 1:6), guidanceParams);
 R = guidanceParams.angleOffset;
 burnVels2d = burnVels2d * R;
 
 vels2d = burnVels2d * guidanceParams.firstDetectionTime / timeToCollision;
 stateTime = stateTime * timeToCollision / maneuver(3);
 velDiffs = diff(vels2d,1);
 maneuverEnd = max(find(abs(velDiffs(:, 1)) <= 1e-5, 1), find(abs(velDiffs(:, 2)) <= 1e-5, 1));
 if isempty(maneuverEnd)
  maneuverEnd = length(stateTime);
 end
 xpoly = polyfit(stateTime(1:maneuverEnd), vels2d(1:maneuverEnd, 1), 1);
 ypoly = polyfit(stateTime(1:maneuverEnd), vels2d(1:maneuverEnd, 2), 1);
end
