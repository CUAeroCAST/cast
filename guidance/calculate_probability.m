function [pdf, probability] = calculate_probability(estimate, guidanceParams)
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
mu = [estimate.corrState(1), estimate.corrState(2)];
sigma = [estimate.Pcorr(1, 1), estimate.Pcorr(1 ,2); estimate.Pcorr(1, 2), estimate.Pcorr(2, 2)];
rSensor = guidanceParams.rSensor;
stepsize = guidanceParams.discSize;
probability = mvncdf([-rSensor, -rSensor], [rSensor, rSensor], mu, sigma);
if probability > guidanceParams.threshold
    Pcorr = sqrt(estimate.Pcorr);
    xrange = estimate.corrState(1)-3*Pcorr(1, 1) :stepsize : estimate.corrState(1)+3*Pcorr(1, 1);
    yrange = estimate.corrState(2)-3*Pcorr(2, 2): stepsize : estimate.corrState(2)+3*Pcorr(2, 2);
    % Get the mean and sigma for the pdf
    % Calculate the pdf
    % Source: https://www.mathworks.com/help/stats/multivariate-normal-distribution.html
    [x, y] = meshgrid(xrange, yrange);
    X = [x(:) y(:)];
    pdf = mvnpdf(X, mu, sigma);
    [rows, cols] = size(x);
    pdf = reshape(pdf, rows, cols);
else
    pdf = nan;
end
