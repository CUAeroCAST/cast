function [x,y] = find_sat_position(est)
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
Pcorr = est.Ppred;
est.corrState = est.predState;
xrange=est.corrState(1)-3*Pcorr(1,1):1e-1:est.corrState(1)+3*Pcorr(1,1);
yrange=est.corrState(3)-3*Pcorr(3,3):1e-1:est.corrState(3)+3*Pcorr(3,3);
% Find the indices of the satelliet position
indxMax = find(xrange<.01,1,'last'); % the 3s might need to be an input for size of objects
indxMin = find(xrange>-.01, 1);
indyMax = find(yrange<.01, 1, 'last' );
indyMin = find(yrange>-.01, 1);
x = [indxMin indxMax];
y = [indyMin indyMax];
end
