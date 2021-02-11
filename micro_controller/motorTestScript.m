% arduinoObj = serialport('COM4',9600);
t = 0:2.5004e-4:2+2.5004e-4;
halfLength = length(t)/2;
posVec1 = linspace(0,.15,halfLength);
posVec2 = linspace(.15,.45,halfLength);
velVec1 = ones(length(posVec1),1)*.15;
velVec2 = ones(length(posVec1),1)*.3;
xPos = [posVec1';posVec2'];
yPos = [posVec1';posVec2'];
xVel = [velVec1;velVec2];
yVel = [velVec1;velVec2];
xPolynomial = polyfit(t,xPos,4);
yPolynomial = polyfit(t,yPos,4);
vxPolynomial = polyfit(t,xVel,4);
vyPolynomial = polyfit(t,yVel,4);
% sendData(arduinoObj,xPolynomial,yPolynomial,vxPolynomial,vyPolynomial,tMove(1))