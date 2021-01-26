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

fprintf(file, '%i Assertions: %i Failures', assertCount, failCount);
fclose('all');

