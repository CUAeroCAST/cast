function readArduinoData(src, ~)

data = readline(src); %grab line from stream

src.UserData.Data(end+1) = str2double(data); %convert to string and save in ArdData Struct

src.UserData.Count = src.ArdData.Count + 1; %increment count

figure
%take first 1000 data points then plot
if src.UserData.Count > 15
    configureCallback(src, "off");
    plot(src.UserData.Data(2:end));
end

end