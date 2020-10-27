function [pdf,probability] = calculate_probability(estimate)
xrange=estimate.corrState(1)-3*Pcorr(1,1):.001:estimate.corrState(1)+3*Pcorr(1,1);
yrange=estimate.corrState(3)-3*Pcorr(3,3):.001:estimate.corrState(3)+3*Pcorr(3,3);
pdf = zeros(length(xrange),length(yrange));
for i = 1:length(xrange)
    for j = 1:length(yrange)
        pdf(i,j) = exp(((xrange(i)-estimate.corrState(1))^2/(2*Pcorr(1,1)^2))+...
            ((yrange(j)-estimate.corrState(3))^2/(2*Pcorr(3,3)^2)));
    end
end
indx = find(xrange<3 && xrange>-3);
indy = find(yrange<3 && yrange>-3);
if isempty(indx) && inempty(indy)
    probability=0;
else
    % find a good numerical integration method
    probability = 1;
end