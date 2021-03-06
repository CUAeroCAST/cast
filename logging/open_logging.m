function datapath = open_logging(log_data)
 % Generates folder and path for future logging
 % yyyy-mm-dd-HH-MM-SS is a self sorting timestamp for runs
 if log_data
  timenow = datestr(datetime, 'yyyy-mm-dd_HH-MM-SS');

  % strip path down to repo base
  currpath = split(pwd, filesep);
  if ~contains(currpath{end}, 'cast')
   currpath = currpath(1:end-1);
  end
  currpath = join(currpath, filesep);

  % enter 'data' folder and create timestamped folder
  datapath = [currpath{1}, filesep, 'data', filesep, timenow];
  [~, ~] = mkdir(datapath);
  addpath(datapath)
 else
  datapath = '..';
end