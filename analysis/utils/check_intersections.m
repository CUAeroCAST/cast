function no_intersects = check_intersections(refs1, refs2, unit_rels, rad)
% checks for intersections of the affine vectors refs + t*unit_rels
% with sphere of radius rad, used to confirm line of sight
% see
% https://math.stackexchange.com/questions/1939423/calculate-if-vector-intersects-sphere
% for details.
 % calculation done in both directions to exclude false positives
 a = dot(unit_rels, unit_rels, 2);
 b = 2 * dot(unit_rels, refs1, 2);
 c = dot(refs1, refs1, 2) - rad^2;
 disc = b.^2 - 4*a.*c;
 
 b2 = 2*dot(-unit_rels, refs2, 2);
 c2 = dot(refs2, refs2, 2) - rad^2;
 disc2 = b2.^2 - 4*a.*c2;
 
 no_intersects = (disc<0) | (disc2<0);
end