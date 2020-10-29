function log_error(logpath, exception)
 % Log exceptions
 % Used to handle errors without crashing the runtime environment
 % Try-Catch blocks should call this function to timestamp exception
 % and save to a file.
 errorfile = [logpath, filesep, 'errors.txt'];
 fid = fopen(errorfile, 'a+');
 currtime = datestr(datetime, 'HH:MM:SS');
 fprintf(fid, '(TIME) %s :: (ERROR) %s :: (MSG) %s\n',...
        currtime, exception.identifier, exception.message);
 fclose(fid);
end