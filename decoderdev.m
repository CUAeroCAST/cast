clear
clc
close all
importCast;

%dat = readmatrix("expressdata--sendtotrace.txt");
dat = readmatrix("expressdata--sendtotrace.txt");

raw = nan;

angle = [];
distance = [];

for i = 1 : 84 : length(dat)-83
 bits = dat(i:i+83);
 if isstruct(raw)
  [scan, raw] = express_merge(raw, bits);
  angle = [angle, scan.angle];
  distance = [distance, scan.distance];
 else
  raw = express_decode(bits);
 end
end

scatter(distance.*cosd(-angle), distance.*sind(-angle))

