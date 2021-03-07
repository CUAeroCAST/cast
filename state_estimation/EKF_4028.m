syms x xs y ys v1 v2 vx vy
x_vec = [x;y;vx;vy];
y = [sqrt((x-xs)^2 + (y-ys)^2) + v2;
     atan((y-ys)/(x-xs)) + v1];
H = jacobian(y, x_vec)