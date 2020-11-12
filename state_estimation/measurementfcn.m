function y = measurementfcn(x)
% Measurement function takes in state and returns range and bearing
% measurement. H is the measurement matrix as defined by sensor
H = [0,1,0,0;
     1,0,0,0];
y = H*x;
end