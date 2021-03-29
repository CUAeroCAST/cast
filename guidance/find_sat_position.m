function [x,y] = find_sat_position(estimate, guidanceParams)
% Finds the satellite position in the pdf 
% Uses the state estimate and covariance to find the indices of the satellites position
% in the pdf
% Inputs:
% estimate: the estimated state and covariance of the debris
% Outputs:
% x: x indice of the satellite position
% y: y indice of the satellite position
% Author: Jason Balke | Project: CAST | Date: 10/29/20
%----------------------------------------------------------------------------------------%
% Create the range for the pdf
stepsize = guidanceParams.discSize;

Pcorr = estimate.Ppred;
estimate.corrState = estimate.predState;

xrange = estimate.corrState(1)-3*Pcorr(1, 1) : stepsize : estimate.corrState(1)+3*Pcorr(1, 1);
yrange = estimate.corrState(2)-3*Pcorr(2, 2) : stepsize : estimate.corrState(2)+3*Pcorr(2, 2);
% Find the indices of the satelliet position
x= find(abs(xrange)== min(abs(xrange)));
y = find(abs(yrange) == min(abs(yrange)));
end
