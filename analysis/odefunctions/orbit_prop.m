function ydot = orbit_prop(t, y, mu)
% propogates the undisturbed two body equation
    r = y(1:3);
    v = y(4:6);
    
    rdot = v;
    vdot = -mu * r / norm(r)^3;

    ydot = [rdot; vdot];
end