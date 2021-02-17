#include <math.h>

union {
    byte asBytes[4];
    float asNum;
 } floatPointNum;

int xEA_minus = 9;
int xEA_plus = 8;
int xEB_minus = 7;
int xEB_plus = 6;
float xPos = 0;
float yPos = 0;
int xaState;
int xaLastState;
int xcounter = 0;
int xDriverPUL = 2;
int xDriverDIR = 3;
int yDriverPUL = 10;
int yDriverDIR = 11;
float a4x = 0;
float a4y = 0;
float a4vx = 0;
float a4vy = 0;

float a3x = 0;
float a3y = 0;
float a3vx = 0;
float a3vy = 0;

float a2x = 0;
float a2y = 0;
float a2vx = 0;
float a2vy = 0;

float a1x = 0;
float a1y = 0;
float a1vx = 0;
float a1vy = 0;

float a0x = 0;
float a0y = 0;
float a0vx = 0;
float a0vy = 0;
float currentTime;
float timestep = 2.5004e-4;

float ratio = 2.7929e-4; // Ratio of m/step, need to get real number and make sure its the same for both axis

int xlswitchend = 36;
int xlswitch0 = 38;
int ylswitchend = 40;
int ylswitch0 = 42;
 
void setup() {
  // put your setup code here, to run once:
  
  pinMode(xDriverPUL, OUTPUT);
  pinMode(xDriverDIR, OUTPUT);
  pinMode(yDriverPUL, OUTPUT);
  pinMode(yDriverDIR, OUTPUT);
  
  pinMode(xlswitchend, INPUT);
  pinMode(xlswitch0, INPUT);
  pinMode(ylswitchend, INPUT);
  pinMode(ylswitch0, INPUT);

  pinMode(xEA_minus, INPUT);
  pinMode(xEA_plus, INPUT);
  pinMode(xEB_minus, INPUT);
  pinMode(xEB_plus, INPUT);
  
  Serial.begin(9600);
  int pulseDelay = 100;
  while(digitalRead(xlswitch0)==HIGH){
    digitalWrite(xDriverDIR,LOW);
    digitalWrite(xDriverPUL,HIGH);
    delayMicroseconds(pulseDelay);
    digitalWrite(xDriverPUL,LOW);
    delayMicroseconds(pulseDelay);
  }
  while(digitalRead(ylswitch0)==HIGH){
    digitalWrite(yDriverDIR,LOW);
    digitalWrite(yDriverPUL,HIGH);
    delayMicroseconds(pulseDelay);
    digitalWrite(yDriverPUL,LOW);
    delayMicroseconds(pulseDelay);
  }
  for(int i = 0;i<(0.5/ratio);i++){
    digitalWrite(xDriverDIR,LOW);
    digitalWrite(xDriverPUL,HIGH);
    delayMicroseconds(pulseDelay);
    digitalWrite(xDriverPUL,LOW);
    delayMicroseconds(pulseDelay);
    digitalWrite(yDriverDIR,LOW);
    digitalWrite(yDriverPUL,HIGH);
    delayMicroseconds(pulseDelay);
    digitalWrite(yDriverPUL,LOW);
    delayMicroseconds(pulseDelay);
  }
   xaLastState = digitalRead(xEA_plus);
   // initialize timer1
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 4000;            // preload timer 65536-16MHz/256/2Hz
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}
ISR(TIMER1_OVF_vect){
  TCNT1 = 4000;            // preload time
  Serial.write("Position: ");
  Serial.print(xPos);
  Serial.write("\n");  
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long startMillis;
  unsigned long beforeLoopMillis;
  unsigned long endTime;
  int timecalc;
  startMillis = millis();
  int xSpeed;
  int ySpeed;
  int xNumSteps;
  int yNumSteps;
  float xpos = 0;
  float ypos = 0;

  if(digitalRead(xlswitch0)==LOW)
    xpos = 0;
  if(digitalRead(xlswitchend)==LOW)
    xpos = 1;
  if(digitalRead(ylswitch0)==LOW)
    ypos = 0;
  if(digitalRead(ylswitchend)==LOW)
    ypos = 1;
  
  if (Serial.available()>0){
    float a4x = readInFloat();
    float a4y = readInFloat();
    float a4vx = readInFloat();
    float a4vy = readInFloat();
    
    float a3x = readInFloat();
    float a3y = readInFloat();
    float a3vx = readInFloat();
    float a3vy = readInFloat();
    
    float a2x = readInFloat();
    float a2y = readInFloat();
    float a2vx = readInFloat();
    float a2vy = readInFloat();
     
    float a1x = readInFloat();
    float a1y = readInFloat();
    float a1vx = readInFloat();
    float a1vy = readInFloat();
    
    float a0x = readInFloat();
    float a0y = readInFloat();
    float a0vx = readInFloat();
    float a0vy = readInFloat();

    float startTime = readInFloat();
    currentTime = startTime;
    
    xSpeed = find_speed(a0vx,a1vx,a2vx,a3vx,a4vx,currentTime);
    ySpeed = find_speed(a0vy,a1vy,a2vy,a3vy,a4vy,currentTime);
    xNumSteps = find_steps(a0x,a1x,a2x,a3x,a4x,currentTime);
    yNumSteps = find_steps(a0y,a1y,a2y,a3y,a4y,currentTime);
    beforeLoopMillis = millis();
    timecalc = beforeLoopMillis-startMillis;
    if(digitalRead(xlswitch0)==LOW && digitalRead(xlswitchend)==LOW){
      //command_motor(xNumSteps,xSpeed,xDriverDIR,xDriverPUL);
    }
      
    if(digitalRead(xlswitch0)==LOW && digitalRead(xlswitchend)==LOW){
      //command_motor(yNumSteps,ySpeed,yDriverDIR,yDriverPUL);
    }
  }
  else if(a4x==0 && a3x==0){
    a4x==0;
  }
  else{
    xSpeed = find_speed(a0vx,a1vx,a2vx,a3vx,a4vx,currentTime);
    ySpeed = find_speed(a0vy,a1vy,a2vy,a3vy,a4vy,currentTime);
    xNumSteps = find_steps(a0x,a1x,a2x,a3x,a4x,currentTime);
    yNumSteps = find_steps(a0y,a1y,a2y,a3y,a4y,currentTime);

    Serial.print(xSpeed);
    Serial.print(ySpeed);
    Serial.print(xNumSteps);
    Serial.print(yNumSteps);
    beforeLoopMillis = millis();
    timecalc = beforeLoopMillis-startMillis;

    if(digitalRead(xlswitch0)==LOW && digitalRead(xlswitchend)==LOW){
      //command_motor(xNumSteps,xSpeed,xDriverDIR,xDriverPUL);
    }
      
    if(digitalRead(xlswitch0)==LOW && digitalRead(xlswitchend)==LOW){
      //command_motor(yNumSteps,ySpeed,yDriverDIR,yDriverPUL);
    }
  }
  
  xaState = digitalRead(xEA_plus); // Reads the "current" state of the outputA
  int xbState = digitalRead(xEB_plus);
  // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (xaState != xaLastState){
    // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (digitalRead(xEB_plus) != xaState) {
      xcounter ++; //cw
    } else {
      xcounter --; //ccw
    }
    xPos = float(counter)/1000;
  }
  xaLastState = xaState; // Updates the previous state of the outputA with the current state
  
  currentTime = currentTime+timestep;
  endTime = millis();
  /*if (endTime-startTime<timestep*1000){
    delayMicroseconds(timestep*1000000-(endTime-startTime)*1000)
  }*/
}

float readInFloat(){
  for(int i=0;i<4;i++){
     floatPointNum.asBytes[i] = Serial.read();
   }
  return floatPointNum.asNum;
}

int find_speed(float a0,float a1,float a2,float a3,float a4,float currentTime){
  float ratio = 2.7929e-4;
  float vel = a4*pow(currentTime,4)+a3*pow(currentTime,3)+a2*pow(currentTime,2)+a1*currentTime+a0;
  int stepSpeed = round(vel/ratio);
  //stepSpeed = stepSpeed*(timestep/(timestep-tcalc));  ADD timecalc to inputs if this line needs to go in
  return stepSpeed;
}

int find_steps(float a0,float a1,float a2,float a3,float a4,float currentTime){
  float ratio = 2.7929e-4;
  float t1 = currentTime+2.5004e-4;
  float dist0 = a4*pow(currentTime,4)+a3*pow(currentTime,3)+a2*pow(currentTime,2)+a1*currentTime+a0;
  float dist1 = a4*pow(t1,4)+a3*pow(t1,3)+a2*pow(t1,2)+a1*t1+a0;
  float dist = dist1-dist0;
  int numSteps = round(dist/ratio);
  return numSteps;
}

void command_motor(int numSteps,int stepSpeed,int driverDIR,int driverPUL){
  
  int pulseDelay = 1/abs(stepSpeed);
  boolean dir;
  if (stepSpeed>0){
    dir = HIGH;
  }
  else{
    dir = LOW;
  }
  // make sure the amount of time this loop runs for is right
  for (int i = 0;i<=numSteps;i++){
    digitalWrite(driverDIR,dir);
    digitalWrite(driverPUL,HIGH);
    delayMicroseconds(pulseDelay);
    digitalWrite(driverPUL,LOW);
    delayMicroseconds(pulseDelay);
  }
}
