function T = orbit_period(mu, a)
% calculates orbital period
    T = 2 * pi * sqrt(a^3/mu);
end