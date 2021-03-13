function send_commands(arduinoObj,x0,x1,y0,y1,xstop,ystop)

write(arduinoObj,single(x0),"single");
write(arduinoObj,single(x1),"single");
write(arduinoObj,single(y0),"single");
write(arduinoObj,single(y1),"single");
write(arduinoObj,single(xstop),"single");
write(arduinoObj,single(ystop),"single");

end