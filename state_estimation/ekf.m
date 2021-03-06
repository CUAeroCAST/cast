function [estimate,estimatorParams,H,yk,y_pred] = ekf(sensorReading,time,estimatorParams,xs,ys)
yk = [sensorReading(2);sensorReading(1)];
% figure(2)
% polarplot(sensorReading(2),sensorReading(1),'*')
% hold on
deltat = time - estimatorParams.currentTime;
estimatorParams.currentTime = time;
xk = estimatorParams.filter.State;
Pk = estimatorParams.filter.StateCovariance;
% Q = estimatorParams.filter.ProcessNoise(deltat);
Q = .01*eye(4);
R = estimatorParams.sensorCovariance;
A = [0 0 1 0;0 0 0 1;0 0 0 0;0 0 0 0];
Omega = [deltat^2/2 0;0 deltat^2/2;deltat 0; 0 deltat];
F = eye(4)+A*deltat;
predState = F*xk;
Ppred = F*Pk*F'+Q;
x = xk(1);
y = xk(2);
H = [-(y - ys)/((x - xs)^2*((y - ys)^2/(x - xs)^2 + 1)),1/((x - xs)*((y - ys)^2/(x - xs)^2 + 1)),0,0;
      (2*x - 2*xs)/(2*((x - xs)^2 + (y - ys)^2)^(1/2)),(2*y - 2*ys)/(2*((x - xs)^2 + (y - ys)^2)^(1/2)),0,0];
if(~any(isnan(yk)))
    Kk = Ppred*H'*inv(H*Ppred*H'+R);
    y_pred = [atan2((y-ys),(x-xs));sqrt((x-xs)^2+(y-ys)^2)];
    corrState = predState+Kk*(yk-y_pred);
    Pcorr = (eye(4)-Kk*H)*Ppred*(eye(4)-Kk*H)'+Kk*R*Kk';
else
    corrState = predState;
    Pcorr = Ppred;
end
estimate.predState = predState;
estimate.Ppred = Ppred;
estimate.corrState = corrState;
estimate.Pcorr = Pcorr;
estimatorParams.filter.StateCovariance = estimate.Pcorr;
estimatorParams.filter.State = estimate.corrState;
% Plots for debugging
% plot(yk(2)*cos(yk(1)),yk(2)*sin(yk(1)),'*');
% hold on
% plot(predState(1),predState(2),'o')
end