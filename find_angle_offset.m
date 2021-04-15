function angleOffset = find_angle_offset(guidanceParams)
 satelliteState = guidanceParams.chiefState;
 unitRad = satelliteState(1:3) / norm(satelliteState(1:3));
 unitAlong = satelliteState(4:6) / norm(satelliteState(4:6));
 unitCross = cross(unitRad, unitAlong);
 relPos = guidanceParams.deputyState(1:3) - satelliteState(1:3);
 relPos = relPos / norm(relPos);
 relPos = relPos - dot(relPos, unitRad) * unitRad;
 angleOffset = acos(dot(relPos, unitAlong));
 relPosC = dot(unitCross, relPos);
 if relPosC > 0
  angleOffset = -angleOffset;
 end
 angleOffset = [cos(angleOffset), -sin(angleOffset);
                sin(angleOffset),  cos(angleOffset)];
end