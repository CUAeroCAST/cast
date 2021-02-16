clear
clc
close all

global file;
global failCount;
global assertCount;
global lineNum;

importCast;

datapath = open_logging(true);
file = fopen([datapath, filesep, 'test.txt'], 'w');
lineNum = @(stack) stack.line;
failCount = 0;
assertCount = 0;

test_state_estimation;
test_guidance;

test_live_plot;

fprintf(file, '%i Assertions: %i Failures', assertCount, failCount);
pause(1);
fclose('all');
