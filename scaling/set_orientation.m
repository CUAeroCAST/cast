function orientation = set_orientation(target)
 chiefVec = [-1, 0, 0];
 relVec = -target.Position;
 rot = cross(chiefVec, relVec);
 u = rot / norm(rot);
 alpha = atan2(norm(rot), dot(chiefVec, relVec));
 qReal = cos(alpha/2);
 qImag = sin(alpha/2)*u;
 quat = quaternion(qReal, qImag(1), qImag(2), qImag(3)); 
 orientation = rad2deg(euler(quat, "ZYX", "point"));
end