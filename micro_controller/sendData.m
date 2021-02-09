function [] = sendData(arduinoObj,xPolynomial,yPolynomial,vxPolynomial,vyPolynomial,tStart)
%Function for sending data to Arduino

% Might be a better way of doing this without a loop, I couldn't find any
for i = 1:length(xPolynomial)
    write(arduinoObj,xPolynomial(i),"single");
    write(arduinoObj,',',"ascii");
    write(arduinoObj,yPolynomial(i),"single");
    write(arduinoObj,',',"ascii");
    write(arduinoObj,vxPolynomial(i),"single");
    write(arduinoObj,',',"ascii");
    write(arduinoObj,vyPolynomial(i),"single");
    write(arduinoObj,',',"ascii");
end
write(arduinoObj,tStart,"single");
% Message has been sent, now send line terminators
write(arduinoObj,'\r\n',"ascii");
end

