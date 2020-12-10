function [pdf,probability,sigx,sigy] = calculate_probability(estimate)
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

%%%%%%%%%%%%%%%%%%%
%error where corrected state is not passed in, predicted used instead
estimate.Pcorr = estimate.Ppred;
estimate.corrState = estimate.predState;
%%%%%%%%%%%%%%%%%%%
mu = [estimate.corrState(1) estimate.corrState(3)];
sigma = [estimate.Pcorr(1,1) estimate.Pcorr(1,3);estimate.Pcorr(1,3) estimate.Pcorr(3,3)];
rSensor = .1011;
probability = mvncdf([-rSensor -rSensor],[rSensor rSensor],mu,sigma);
if probability>.25
    Pcorr = sqrt(estimate.Pcorr);
    xrange=estimate.corrState(1)-3*Pcorr(1,1):1e-1:estimate.corrState(1)+3*Pcorr(1,1);
    yrange=estimate.corrState(3)-3*Pcorr(3,3):1e-1:estimate.corrState(3)+3*Pcorr(3,3);
    % Get the mean and sigma for the pdf
    % Calculate the pdf
    % Source: https://www.mathworks.com/help/stats/multivariate-normal-distribution.html
    [x,y] = meshgrid(xrange,yrange);
    X = [x(:) y(:)];
    pdf = mvnpdf(X,mu,sigma);
    [rows,cols] = size(x);
    pdf = reshape(pdf,rows,cols);
    % probability the debris is in a 3 m box around the satellite
    indxMax = find(xrange<rSensor,1,'last');
    indxMin = find(xrange>-rSensor, 1);
    indyMax = find(yrange<rSensor, 1, 'last' );
    indyMin = find(yrange>-rSensor, 1);
    % If the satellie is not in the range of the pdf, probabilty is 0, else
    % calculate the probability
    sigx = [estimate.corrState(1)-2*Pcorr(1,1) estimate.corrState(1)+2*Pcorr(1,1)];
    sigy = [estimate.corrState(3)-2*Pcorr(3,3) estimate.corrState(3)+2*Pcorr(3,3)];
else
    sigx = 0;
    sigy = 0;
    pdf = 0;
end
