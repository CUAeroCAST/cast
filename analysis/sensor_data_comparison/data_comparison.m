close all
clear 
clc

%% Headon Scenario
load('Headon.mat');
data1_live = load('livedata--1-5m--headon--2ndbulb--march8.txt');
data1_live(:,1) = data1_live(:,1) - data1_live(1,1);
data1_live(:,2) = data1_live(:,2)/1000; %covert to meters
data1(:,3) = deg2rad(data1(:,3)); %convert to radians
data1_live(:,3) = deg2rad(data1_live(:,3)); %convert to radians

%Averging generated data
data1(:,1) = round(data1(:,1),2);
[C,~,idx] = unique(data1(:,1),'stable');
data1_avg(:,1) = C;
data1_avg(:,2) = accumarray(idx,data1(:,2),[],@mean); 
data1_avg(:,3) = accumarray(idx,exp(1i*data1(:,3)),[],@mean); 
data1_avg(:,3) = atan2(imag(data1_avg(:,3)),real(data1_avg(:,3)));

%Plotting
x1_gen = data1_avg(:,2).*cos(data1_avg(:,3));
y1_gen = data1_avg(:,2).*sin(data1_avg(:,3));
x1_live = data1_live(:,2).*cos(data1_live(:,3));
y1_live = data1_live(:,2).*sin(data1_live(:,3));

figure
hold on
plot(x1_gen,y1_gen,'x')
plot(x1_live,y1_live,'x')
plot(0,0,'O','LineWidth',2)
legend("Generated Data", "Live Data", "Sensor Location")
ylabel("Y [m]")
xlabel("X [m]")
xlim([-0.1,1.5])
ylim([-0.1,0.1])

%Statistics
sig1_gen = rad2deg(std(data1_avg(:,3)));
sig1_live = rad2deg(std(data1_live(:,3)));
fprintf("Generated Data Headon Angle std = %.2f deg\n", sig1_gen);
fprintf("Live Data Headon Angle std = %.2f deg\n", sig1_live);

%% Near Miss
load('nearMiss.mat')
data2_live = load('livedata--1-5m--nearmiss--2ndbulb--march8.txt');
data2_live(:,1) = data2_live(:,1) - data2_live(1,1);
data2_live(:,2) = data2_live(:,2)/1000; %covert to meters
data2(:,3) = deg2rad(data2(:,3)); %convert to radians

data2(:,1) = round(data2(:,1),2);
[C,~,idx] = unique(data2(:,1),'stable');
data2_avg(:,1) = C;
data2_avg(:,2) = accumarray(idx,data2(:,2),[],@mean); 
data2_avg(:,3) = accumarray(idx,exp(1i*data2(:,3)),[],@mean); 
data2_avg(:,3) = atan2(imag(data2_avg(:,3)),real(data2_avg(:,3)));

%Plotting
x2_gen = data2_avg(:,2).*cos(data2_avg(:,3));
y2_gen = data2_avg(:,2).*sin(data2_avg(:,3));
x2_live = data2_live(:,2).*cos(data2_live(:,3));
y2_live = data2_live(:,2).*sin(data2_live(:,3));

figure
hold on
plot(x2_gen,y2_gen,'x')
plot(x2_live,y2_live,'x')
plot(0,0,'O','LineWidth',2)
legend("Generated Data", "Live Data", "Sensor Location")
ylabel("Y [m]")
xlabel("X [m]")
xlim([-0.1,1.5])
ylim([-0.1,0.25])

%statistics
sig2_gen = rad2deg(std(data2_avg(:,3)));
sig2_live = rad2deg(std(data2_live(:,3)));
fprintf("Generated Data Near Miss Angle std = %.2f deg\n", sig2_gen);
fprintf("Live Data Near Miss Angle std = %.2f deg\n", sig2_live);

%% Clear Miss
load('clearMiss.mat')
data3_live = load('livedata--1-5m--267mm-offset--bigmiss--2ndbulb--march8.txt');
data3_live(:,1) = data3_live(:,1) - data3_live(1,1);
data3_live(:,2) = data3_live(:,2)/1000; %covert to meters
data3(:,3) = deg2rad(data3(:,3)); %convert to radians

data3(:,1) = round(data3(:,1),2);
[C,~,idx] = unique(data3(:,1),'stable');
data3_avg(:,1) = C;
data3_avg(:,2) = accumarray(idx,data3(:,2),[],@mean); 
data3_avg(:,3) = accumarray(idx,exp(1i*data3(:,3)),[],@mean); 
data3_avg(:,3) = atan2(imag(data3_avg(:,3)),real(data3_avg(:,3)));

%Plotting
x3_gen = data3_avg(:,2).*cos(data3_avg(:,3));
y3_gen = data3_avg(:,2).*sin(data3_avg(:,3));
x3_live = data3_live(:,2).*cos(data3_live(:,3));
y3_live = data3_live(:,2).*sin(data3_live(:,3));

figure
hold on
plot(x3_gen,y3_gen,'x')
plot(x3_live,y3_live,'x')
plot(0,0,'O','LineWidth',2)
legend("Generated Data", "Live Data", "Sensor Location")
ylabel("Y [m]")
xlabel("X [m]")
xlim([-0.1,1.5])
ylim([-0.1,0.5])

%Statistics
sig3_gen = rad2deg(std(data3_avg(:,3)));
sig3_live = rad2deg(std(data3_live(:,3)));
fprintf("Generated Data Clear Miss Angle std = %.2f deg\n", sig3_gen);
fprintf("Live Data Clear Miss Angle std = %.2f deg\n", sig3_live);