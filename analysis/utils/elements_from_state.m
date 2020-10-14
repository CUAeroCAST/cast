function [true_anomaly,...
          semi_major,...
          arg_of_peri,...
          right_ascension,...
          eccentricity,...
          inclination] = elements_from_state(state, mu)
% computes the orbital elements from the state vector
% see Curtis, Orbital Mechanics for Engineering Student pg 197.
      pos = state(1:3);
      vel = state(4:6);
      
      r = norm(pos);
      v = norm(vel);
      
      rad_vel = dot(pos, vel) / r;
      
      ang_mom = cross(pos, vel);
      h = norm(ang_mom);
      
      inclination = acosd(ang_mom(3) / h);
      
      node_line = cross([0; 0; 1], ang_mom);
      N = norm(node_line);
      
      right_ascension = acosd(node_line(1) / N);
      if node_line(2) < 0
          right_ascension = 360 - right_ascension;
      end
      
      ecc_vec = ((v^2 - mu/r)*pos - r*rad_vel*vel) / mu;
      eccentricity = norm(ecc_vec);
      
      arg_of_peri = acosd(dot(node_line/N, ecc_vec/eccentricity));
      if ecc_vec(3) < 0
          arg_of_peri = 360 - arg_of_peri;
      end
      
      true_anomaly = acosd((h^2/mu/r - 1) / eccentricity);
      if rad_vel < 0
          true_anomaly = 360 - true_anomaly;
      end
      
      rp = h^2/mu / (1 + eccentricity);
      ra = h^2/mu / (1 - eccentricity);
      semi_major = (rp + ra) / 2;
end