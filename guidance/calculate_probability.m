function [pdf,probability,pError] = calculate_probability(estimate)
xrange=estimate.corrState(1)-4*Pcorr(1,1):.001:estimate.corrState(1)+4*Pcorr(1,1);
yrange=estimate.corrState(3)-4*Pcorr(3,3):.001:estimate.corrState(3)+4*Pcorr(3,3);
mu = [estimate.corrState(1) estimate.corrState(3)];
sigma = [estimate.Pcorr(1,1) estimate.Pcorr(1,3);estimate.Pcorr(1,3) estimate.Pcorr(3,3)];
[x,y] = meshgrid(xrange,yrange);
X = [x(:) y(:)];
pdf = mvnpdf(X,mu,sigma);
pdf = reshape(pdf,length(x),length(y));
% pdf = zeros(length(xrange),length(yrange));
% for i = 1:length(xrange)
%     for j = 1:length(yrange)
%         pdf(i,j) = exp(((xrange(i)-estimate.corrState(1))^2/(2*Pcorr(1,1)^2))+...
%             ((yrange(j)-estimate.corrState(3))^2/(2*Pcorr(3,3)^2)));
%     end
% end
indxMax = find(xrange<3,1,'last'); % the 3s might need to be an input for size of objects
indxMin = find(xrange>-3, 1);
indyMax = find(yrange<3, 1, 'last' );
indyMin = find(yrange>-3, 1);
if isempty(indx) && inempty(indy)
    probability=0;
else
    [probability,pError] = mvncdf([indxMin indxMax],[indyMin indyMax],mu,sigma);
end