function sph_error = sphError(sphere_coords, chief_radius, relVels)
 rho = sphere_coords(1, :);
 theta = sphere_coords(2, :) / chief_radius;
 phi = sphere_coords(3, :);
 
 r = rho + chief_radius;
 x = r .* cos(theta) .* cos(phi) - chief_radius;
 y = r .* sin(theta) .* cos(phi);
 z = r .* sin(phi);
 
 sph_error = vecnorm(cross([x; y; z], relVels));
end