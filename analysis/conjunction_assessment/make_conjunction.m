function state = make_conjunction(altitude, mu, rad)
% generates a random polar orbit
 r = [0; 0; altitude + rad];
 solved = 0;
 v_guess = sqrt(mu/rad);
 while solved < 1
  ecc_target = 0; % rand();
  ecc = @(v) norm(((dot(v, v) - mu / norm(r))*r - dot(r, v) * v)/mu)...
   - ecc_target;
  options = optimoptions(@fsolve, 'Algorithm', 'levenberg-marquardt', ...
   'Display', 'off');
  signs = randi(2, 3, 1) - 1;
  signs(~signs) = -1;
  signs(1) = -1;
  V_guess = [0; v_guess; 0].*signs + randn(3,1).*[v_guess/3; v_guess/3; 0.1];
  [V, ~, solved] = fsolve(ecc, V_guess, options);
 end
 state = [r; V];
end