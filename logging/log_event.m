function log_event(logpath, event)
 % Log events
 % Utilizes matlabs exceptions as event messages.
 % Generate expections with event = MException('module:eventname','message')
 % and call this function to log a timestamped event.
 eventsfile = [logpath, filesep, 'events.txt'];
 fid = fopen(eventsfile, 'a+');
 currtime = datestr(datetime, 'HH:MM:SS');
 fprintf(fid, '(TIME) %s :: (EVENT) %s :: (MSG) %s\n',...
        currtime, event.identifier, event.message);
 fclose(fid);
end