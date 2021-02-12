clear
clc
close all

blah()
function blah()
filename = "data.txt";
writematrix(["time(ms)", "start", "qual", "angle(deg)", "distance(mm)"], filename);
sensorObj = serial_Sensor();
cleanup = onCleanup(@()clean_up(sensorObj));
%%
start_Motor(sensorObj);
SR = sampling_Rate(sensorObj);
scan_Request(sensorObj);
j = 0;
data = struct;
time = 0;
while true
    pause(1e-6)
    %disp(sensorObj);
 n = 1;
 if sensorObj.UserData.dataReady
  for i = 1:numel(sensorObj.UserData.scan)
      if j>=7
          if mod(j-7,5) == 0
              data.time(n) = time;
              data.start(n) = mod(sensorObj.UserData.scan(i),3);
              data.qual(n) = sensorObj.UserData.scan(i) - data.start(n);
          elseif mod(j-7, 5) == 1
              data.angle(n) = typecast(uint8([sensorObj.UserData.scan(i), bitshift(sensorObj.UserData.scan(i+1),-1)]),'uint16')/64;
          elseif mod(j-7, 5) == 3
              data.distance(n) = typecast(uint8([sensorObj.UserData.scan(i), sensorObj.UserData.scan(i+1)]),'uint16')/4;
              n = n+1;
              time = time + 1/(2); % milliseconds
          end
      end
      j = j + 1;
  end
  if j>7
      datamat = [data.time', data.start', data.qual', double(data.angle'), double(data.distance')];
      writematrix(datamat, filename, 'WriteMode', 'append');
  end
  sensorObj.UserData.dataReady = false;
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

