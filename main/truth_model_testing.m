%% HOUSEKEEPING
clear; clc; close all
importCast;

%% VALUES TO CHANGE FOR TMT
simulationParams.initPos = [1.5,0.5,0]; %m, starting point of object
simulationParams.finalPos = [0,0,0]; %m, final point of object
simulationParams.collisionTime = 1.5; %s, time it takes to get from initial 
simulationParams.Q = (1/100).*eye(4); %True value of process noise, used for adding noise to truth states

estimatorParams.sensorCovariance = [0.025^2,0;0,deg2rad(0.45)^2]; %Range, bearing
estimatorParams.qGain = 1; %Process noise gain for forward prediction
estimatorParams.initState = [1.5;-1;0;0]; %Constant x-vel init state

%% OTHER CONFIG (Don't Tweak)
%Simulation parameters (for sensor model)
simulationParams.stepSize = 1;
simulationParams.sampleRate = 4e3;
simulationParams.scalingFactor = 1; %Distance scaling factor for testbed
%to final position

%Estimator parameters
estimatorParams.llsSeeding = false;
estimatorParams.batchSamples = 0;

%Sensor parameters
sensorParams.samplingRate = 4e3;
sensorParams.maxRange = 4e3;
sensorParams.beamDivergence = 0.9; %deg
sensorParams.rangeAccuracy = 0.025; %m
sensorParams.beamLimits = [-1.35,1.35];
sensorParams.sensorType = 'Lidar';
sensorParams.scanRate = 10; %Hz

%Target parameters
targetParams.Mesh = extendedObjectMesh('sphere');
targetParams.Dimensions.Length = 50e-3; 
targetParams.Dimensions.Width = 50e-3;
targetParams.Dimensions.Height = 50e-3;
targetParams.Dimensions.OriginOffset = [0,0,0];

%% TESTBED COLLISION
x_rel = linspace(simulationParams.initPos(1),...
             simulationParams.finalPos(1),...
             simulationParams.collisionTime*simulationParams.sampleRate);
y_rel = linspace(simulationParams.initPos(2),...
             simulationParams.finalPos(2),...
             simulationParams.collisionTime*simulationParams.sampleRate);
z_rel = linspace(simulationParams.initPos(3),...
             simulationParams.finalPos(3),...
             simulationParams.collisionTime*simulationParams.sampleRate);
relativePath = [x_rel', y_rel', z_rel'];
timeVec = linspace(0,simulationParams.collisionTime, ...
        simulationParams.collisionTime*simulationParams.sampleRate);

%% STASTICAL STUFF
simulationParams.xvel = (simulationParams.finalPos(1) - simulationParams.initPos(1))/simulationParams.collisionTime;
simulationParams.yvel = (simulationParams.finalPos(2) - simulationParams.initPos(2))/simulationParams.collisionTime;



N=1;
% ykrec=zeros(2,130);
% yk_minusrec=zeros(2,130);
xktruerec=zeros(4,6000);
xk_plusrec=zeros(4,6000);
Pk_plusrec=cell(N,6000);
% Pk_minusrec=zeros(4,520);
% Rrec=zeros(2,2);

c=1;



for j=1:N

x_true = [x_rel;
          simulationParams.xvel*ones(1,length(timeVec));
          y_rel;
          simulationParams.yvel*ones(1,length(timeVec))];
process_noise = mvnrnd([0,0,0,0], simulationParams.Q, length(timeVec))';
x_true = x_true + process_noise;
%add x-dir process noise to relative path
relativePath(:,1) = relativePath(:,1) + process_noise(1,:)';
%add y-dire process noise to relative path
relativePath(:,2) = relativePath(:,3) + process_noise(3,:)';
%% SENSOR MODEL
%If sensor readings/time vec is saved, can load that to save time
sensorScenario = init_sensor_model(relativePath, timeVec, sensorParams,...
                                   targetParams);
sensorReadings = sensor_model(sensorScenario, false);

%Constants for converting measurement to cartesian
lam = lam_vals(estimatorParams.sensorCovariance);
%% STATE ESTIMATION
%Determine how many sensor readings to use for batch LLS estimate
[offset, estimatorParams] = init_estimator(sensorReadings, estimatorParams);
estimatorParams.currentTime = timeVec(offset);

%% MAIN LOOP
for i = 2 : simulationParams.stepSize : length(timeVec)
    % STATE ESTIMATION
    sensorReading = sensorReadings(i,:);
    time = timeVec(i);
    if(~any(isnan(sensorReading)))
        %Convert range-bearing to xy
        mu = conv_meas_bias(lam, sensorReading);
        sensorReading = meas2cart(sensorReading, mu);

        %Account for conversion bias
        if(~any(isnan(sensorReading)))
            R_conv = get_conv_cov(estimatorParams.sensorCovariance, lam, sensorReading);
            estimatorParams.filter.MeasurementNoise = R_conv;
        end
    end
    %Estimate the state
    [estimate, estimatorParams] = state_estimator(sensorReading, time,...
                                                  estimatorParams);
    %Save off important values for truth model testing
    xp(:,i) = estimate.predState;
    xc(:,i) = estimate.corrState;
    
    Pp{i} = estimate.Ppred;
    Pc{i} = estimate.Pcorr;
    
    %     if ~any(isnan(sensorReading'))
%     yki(:,c)=sensorReading';
%     vartest=[xp(1,c);xp(3,c)];
%     ykmi(:,c)=vartest;
%     Ri{c}=R_conv;
%     Ppi{c} = estimate.Ppred;
%     Pci{c} = estimate.Pcorr;
%     x_truei(:,c)=x_true(:,i);
%     
%     c=c+1;
%     end

x_truei(:,i)=x_true(:,i);
Ppi{i} = estimate.Ppred;
Pk_plusrec{j,i}=Ppi{i};
end

%     R{j}=Ri{:};
    xktrue{j}=x_truei;
    Pk_plus{j}=Ppi;
    xk_plus{j}=xp;
%     Pk_minus{j}=cell2mat(Ppi);
%     yk{j}=yki;
%     yk_minus{j}=ykmi;
%     
% ykrec=ykrec+yk{j};
% yk_minusrec=yk_minusrec+yk_minus{j};
% Pk_minusrec=Pk_minusrec+Pk_minus{j};
xktruerec=xktruerec+xktrue{j};
xk_plusrec=xk_plusrec+xk_plus{j};


% Rrec=Rrec+R{j};

if j<N
    
    clear xktrue
    clear Pk_plus
    clear xk_plus
%     clear Ri
%     clear Pk_minus
%     clear yk
end
end

Pk_plusrec{1,1}=zeros(4,4); %%%% Shouldn't need to do this ... hacky ...
Pk_plusadd=cell(1,length(Pk_plusrec));
Nf=N;
for j=1:length(Pk_plusrec)
    
    Pk_plusadd{j}=zeros(4,4);
    
    
    for i=1:N
        
    if isempty(Pk_plusrec{i,j})==1
        
        Pk_plusrec{i,j}=zeros(4,4);
        
    end
    
    Pk_plusadd{j}=Pk_plusadd{j}+Pk_plusrec{i,j};
    
    end
    
    Pk_plusMean{j}=Pk_plusadd{j}./Nf;
end
xktrue=xktruerec./N;
xk_plus=xk_plusrec./N;

tvec=timeVec;

H=[1,0,0,0;0,0,1,0];

%Use these variables for each trial
%xtrue, timeVec, xp,xc,Pp,Pc, sensorReadings
%test_NIS( yk, yk_minus, Pk_minus,H,R, tvec, N, alpha )
%test_NEES( xktrue, xk_plus, Pk_plus, tvec, N ,alpha )

%% Calling NEES and NIS tests

n=4;
p=2;

% For 70 % confidence
alpha=0.3;
% test_NIS( yk, yk_minus, Pk_minus,H,R, tvec, N, n, alpha )
%figure;
test_NEES( xktrue, xk_plus, Pk_plusMean, tvec, N, p, alpha )

% For 95 % confidence
alpha=0.05;
% test_NIS( yk, yk_minus, Pk_minus,H,R, tvec, N,n, alpha )
%figure;
test_NEES( xktrue, xk_plus, Pk_plusMean, tvec, N ,p,alpha )