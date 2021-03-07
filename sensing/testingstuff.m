clear
clc
close all

blah()
function blah()
filename = "data.txt";
writematrix(["time(ms)", "start", "qual", "angle(deg)", "distance(mm)"], filename);

sensorParams = struct;
sensorParams.readsize = 2000;
sensorParams.scanMode = "standard";
sensorParams.portstr = "/dev/tty.usbserial-0001";

sensorParams.sensorObj = serial_Sensor(sensorParams);
cleanup = onCleanup(@()clean_up(sensorParams.sensorObj));
%%
% start_Motor(sensorObj);
SR = sampling_Rate(sensorParams.sensorObj);
scan_Request(sensorParams);

j = 0;
while true
    pause(1e-6)
    %disp(sensorObj);
 if sensorParams.sensorObj.UserData.dataReady
   data = sensorParams.sensorObj.UserData.scan;
   datamat = [data.time', data.start', data.qual', double(data.angle'), double(data.distance')];
   writematrix(datamat, filename, 'WriteMode', 'append');
   sensorParams.sensorObj.UserData.dataReady = false;
  end
  
 end
end

function clean_up(sensorObj)
 stop_Motor(sensorObj);
 delete(sensorObj);
end

    %  figure;
    %  scatter(double(data.distance).*cosd(double(data.angle)), double(data.distance).*sind(double(data.angle)),'LineWidth',3), hold on
    %  title("LiDAR Sensor in Laundry Basket",'fontsize',24,'Interpreter','latex')
    %  xlabel('x [mm]','fontsize',22,'Interpreter','latex')
    %  ylabel('y [mm]','FontSize',24,'Interpreter','latex')
    %  set(gca, 'FontSize', 20)
    %  grid minor;
    
    
%     data=readmatrix("expressdata.txt");
%     figure;
%     scatter(data(:,1).*cosd(data(:,2)), data(:,1).*sind(data(:,2)),'LineWidth',3), hold on
%     title("LiDAR Sensor in Laundry Basket",'fontsize',24,'Interpreter','latex')
%     xlabel('x [mm]','fontsize',22,'Interpreter','latex')
%     ylabel('y [mm]','FontSize',24,'Interpreter','latex')
%     set(gca, 'FontSize', 20)
%     grid minor;

