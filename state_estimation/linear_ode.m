function dx = linear_ode(t,x,w)
%Equation for use with ode45 to integrate path with ZOH process noise
dx = zeros(4,1);
dx(1) = x(2);
dx(2) = 0;
dx(3) = x(4);
dx(4) = 0;
%Add process noise
if isequal(size(dx),size(w))
    dx = dx + w;
else
    try
        dx = dx + w';
    catch
        error('Process noise failed to add')
    end
end

