addpath(genpath('../'))

currpath = split(pwd, filesep);
root = find(strcmp(currpath, 'cast'));
currpath = join(currpath(1:root), filesep);
cd([currpath{1}, filesep, 'main'])
