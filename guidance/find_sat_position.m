function [x,y] = find_sat_position(estimate)
xrange=estimate.corrState(1)-3*Pcorr(1,1):.001:estimate.corrState(1)+3*Pcorr(1,1);
yrange=estimate.corrState(3)-3*Pcorr(3,3):.001:estimate.corrState(3)+3*Pcorr(3,3);
x = find(xrange<.01 && xrange>-.01);
y = find(yrange<.01 && yrange>-.01);
end