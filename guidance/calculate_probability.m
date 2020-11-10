function [pdf,probability,xrange,yrange] = calculate_probability(estimate)
% Calculates probability of collision
% Uses the estimated state and covariance to get a probability density
% function, then calculates probability using the cumulative distribution
% function
% Inputs:
% estimate: estimated state and covariance of the debris
% Outputs:
% pdf: probability distribution function of the debris
% probability: probability of collision
% pError: error of the probability calculation
% Author: Jason Balke | Project: CAST | Date: 10/29/20
%----------------------------------------------------------------------------------------%
% Set the range of the pdf calculations to 4 times the variances
Pcorr = sqrt(estimate.Pcorr);
xrange=estimate.corrState(1)-4*Pcorr(1,1):.000001:estimate.corrState(1)+4*Pcorr(1,1);
yrange=estimate.corrState(3)-4*Pcorr(3,3):.000001:estimate.corrState(3)+4*Pcorr(3,3);
% Get the mean and sigma for the pdf
mu = [estimate.corrState(1) estimate.corrState(3)];
sigma = [estimate.Pcorr(1,1) estimate.Pcorr(1,3);estimate.Pcorr(1,3) estimate.Pcorr(3,3)];
% Calculate the pdf
% Source: https://www.mathworks.com/help/stats/multivariate-normal-distribution.html
[x,y] = meshgrid(xrange,yrange);
X = [x(:) y(:)];
pdf = mvnpdf(X,mu,sigma);
pdf = reshape(pdf,length(x),length(y));
% probability the debris is in a 3 m box around the satellite
indxMax = find(xrange<.1011,1,'last'); % the 3s might need to be an input for size of objects
indxMin = find(xrange>-.1011, 1);
indyMax = find(yrange<.1011, 1, 'last' );
indyMin = find(yrange>-.1011, 1);
% If the satellie is not in the range of the pdf, probabilty is 0, else
% calculate the probability
if isempty(indxMax) && inempty(indy)
    probability=0;
else
    probability = mvncdf([x(indxMin) y(indyMin)],[x(indxMax) y(indyMax)],mu,sigma);
end