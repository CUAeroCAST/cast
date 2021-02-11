#include <math.h>

union {
    byte asBytes[4];
    float asNum;
 } floatPointNum;
 
void setup() {
  // put your setup code here, to run once:
  int xDriverPUL = 7;
  int xDriverDIR = 6;
  int yDriverPUL = 5;
  int yDriverDIR = 4;
  float currentTime;
  float timeStart;
  float timestep = 2.5004e-4;
  float ratio = 2.7929e-4; // Ratio of m/step, need to get real number and make sure its the same for both axis
  Serial.begin(9600);
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
    
    timeStart = readInFloat();
    currentTime = readInFloat();
  }
}



void loop() {
  // put your main code here, to run repeatedly:
  int xDriverPUL = 7;
  int xDriverDIR = 6;
  int yDriverPUL = 5;
  int yDriverDIR = 4;
  int xSpeed;
  int ySpeed;
  int xNumSteps;
  int yNumSteps;

  float a4x;
  float a4y;
  float a4vx;
  float a4vy;

  float a3x;
  float a3y;
  float a3vx;
  float a3vy;

  float a2x;
  float a2y;
  float a2vx;
  float a2vy;

  float a1x;
  float a1y;
  float a1vx;
  float a1vy;

  float a0x;
  float a0y;
  float a0vx;
  float a0vy;
  float currentTime;
  float timestep;
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
    
    xSpeed = find_speed(a0vx,a1vx,a2vx,a3vx,a4vx,currentTime);
    ySpeed = find_speed(a0vy,a1vy,a2vy,a3vy,a4vy,currentTime);
    xNumSteps = find_steps(a0x,a1x,a2x,a3x,a4x,currentTime);
    yNumSteps = find_steps(a0y,a1y,a2y,a3y,a4y,currentTime);
    command_motor(xNumSteps,xSpeed,xDriverDIR,xDriverPUL);
    command_motor(yNumSteps,ySpeed,yDriverDIR,yDriverPUL);
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
    
    //command_motor(xNumSteps,xSpeed,xDriverDIR,xDriverPUL);
    //command_motor(yNumSteps,ySpeed,yDriverDIR,yDriverPUL);
  }
  currentTime = currentTime+timestep;
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
  return stepSpeed;
}

int find_steps(float a0,float a1,float a2,float a3,float a4,float currentTime){
  float ratio = 2.7929e-4;
  float dist = a4*pow(currentTime,4)+a3*pow(currentTime,3)+a2*pow(currentTime,2)+a1*currentTime+a0;
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
