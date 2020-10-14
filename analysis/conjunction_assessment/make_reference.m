function state_vec = make_reference(altitude, mu, rad)
% generates a reference circular, polar orbit
    r = [0; 0; altitude + rad];
    ecc = @(v) ((dot(v, v) - mu / norm(r))*r - dot(r, v) * v)/mu;
    options = optimoptions(@fsolve, 'Display', 'off');
    V = fsolve(ecc, [rad; 0; 0], options);
    V(V < 1e-3) = 0;
    state_vec = [r; V];
end