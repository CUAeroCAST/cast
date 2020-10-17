function curviHillVecs = curvilinear_hill_vecs(hillVecs, chief_radius)
 x = hillVecs(1, :);
 y = hillVecs(2, :);
 z = hillVecs(3, :);
 
 xdt = hillVecs(4, :);
 ydt = hillVecs(5, :);
 zdt = hillVecs(6, :);
 
 gamma = chief_radius + x;
 
 r = -chief_radius + sqrt((chief_radius + x) .^ 2 + y .^ 2 + z .^ 2);
 phi = atan2(z, gamma);
 theta = atan2(y, (gamma .* cos(phi)));
 
 
 rdt = (gamma .* xdt + y .* ydt + z .* zdt) ./ sqrt(gamma .^ 2 + y .^ 2 + z .^ 2);
 phidt = (zdt .* gamma - z .* xdt)...
     ./ (gamma .^ 2 + z .^ 2);
 thetadt = (cos(phi) .* (ydt .* gamma - y .* xdt)...
   + y .* phidt .* sin(phi))...
   ./ (cos(phi) .^ 2 .* gamma .^ 2 + y .^ 2);
 
 
 curviHillVecs = [r;
      chief_radius * theta;
      phi;
      rdt;
      chief_radius * thetadt;
      phidt];
end