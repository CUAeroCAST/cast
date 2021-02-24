function [estimate,estimatorParams] = ekf(yk,time,estimatorParams,xs,ys)
yk = yk';
deltat = time - estimatorParams.currentTime;
estimatorParams.currentTime = time;
xk = estimatorParams.filter.State;
Pk = estimatorParams.filter.StateCovariance;
% Q = estimatorParams.filter.ProcessNoise(deltat);
Q = [.001,0;0,.001];
R = estimatorParams.sensorCovariance;
f = [0 0 1 0;0 0 0 1;0 0 0 0;0 0 0 0];
Omega = [deltat^2/2 0;0 deltat^2/2;deltat 0; 0 deltat];
F = eye(4)+f*deltat;
predState = f*xk;
Ppred = F*Pk*F'+Omega*Q*Omega';
x = xk(1);
y = xk(2);
H = [(x-xs)/((x-xs)^2+(y-ys)^2)^(1/2), (y-ys)/((x-xs)^2+(y-ys)^2)^(1/2),0,0;
    1/((y-ys)*((x-xs)^2/(y-ys)^2+1)), -(x-xs)/((y-ys)^2*((x-xs)^2/(y-ys)^2+1)), 0, 0];
if(~any(isnan(yk)))
    Kk = Ppred*H'*inv(H*Ppred*H'+R);
    y_pred = [atan((x-xs)/(y-ys));sqrt((x-xs)^2+(y-ys)^2)];
    corrState = predState+Kk*(yk-y_pred);
    Pcorr = (eye(4)-Kk*H)*Ppred;
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
end