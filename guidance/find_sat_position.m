function [x,y] = find_sat_position(estimate)
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
Pcorr = estimate.Ppred;
estimate.corrState = estimate.predState;
xrange=estimate.corrState(1)-3*Pcorr(1,1):1e-2:estimate.corrState(1)+3*Pcorr(1,1);
yrange=estimate.corrState(3)-3*Pcorr(3,3):1e-2:estimate.corrState(3)+3*Pcorr(3,3);
% Find the indices of the satelliet position
indxMax = find(xrange<.01,1,'last');
indxMin = find(xrange>-.01, 1);
indyMax = find(yrange<.01, 1, 'last' );
indyMin = find(yrange>-.01, 1);
x = [indxMin indxMax];
y = [indyMin indyMax];
end