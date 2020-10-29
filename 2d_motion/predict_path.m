function [tVec,path] = predict_path(t,hillMat,t0,tStep)
% This function predicts the 2d path of the collision object
% Inputs:
% t: time vector from the original 2D path
% hillMat: 2d state vector in the 2d hill frame
% t0: time of sensor reading
% Outputs:
% path: 2d state vector predicted from the input
% Author: Jason Balke | Project: CAST | Date: 10/20/20
%----------------------------------------------------------------------------------------%
% Finding where the initial time occurs in the state vectors
t0 = t(1)-t0;
ind = find(t==t0);
if isempty(ind)
   ind = find(t<t0,1);
   ind = ind-1;
end
% Getting initial values
initialState = hillMat(:,ind);
initialTime = t(1)-t(ind);
% Create a time vector
tVec = 0:tStep:initialTime;
% Initalizing the state vector
onesVec = ones(1,length(tVec));
path = [onesVec*initialState(1);onesVec*initialState(2)];
% Using kinematic equations of motion to predict the path forward
path(1,:) = path(1,:)+initialState(3).*tVec;
path(2,:) = path(2,:)+initialState(4).*tVec;
end