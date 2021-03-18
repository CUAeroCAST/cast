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
if any(isnan(estimate.corrState), "all")
 Pcorr = estimate.Ppred;
 estimate.corrState = estimate.predState;
else
 Pcorr = estimate.Pcorr;
end
xrange = estimate.corrState(1)-3*Pcorr(1, 1) : stepsize : estimate.corrState(1)+3*Pcorr(1, 1);
yrange = estimate.corrState(2)-3*Pcorr(2, 2) : stepsize : estimate.corrState(2)+3*Pcorr(2, 2);
% Find the indices of the satelliet position
x= find(abs(xrange - 0.01)== min(abs(xrange-0.01)));
indxMin = find(xrange > -0.01, 1);
indyMax = find(yrange < 0.01, 1,  'last' );
indyMin = find(yrange > -0.01, 1);
x = [indxMin indxMax];
y = [indyMin indyMax];
end
