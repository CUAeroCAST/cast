#include <math.h>

  // Defining Pins
  int xDriverPUL = 7;
  int xDriverDIR = 6;
  int yDriverPUL = 5;
  int yDriverDIR = 4;
  
  // Defining Variables
  union {
      byte asBytes[4];
      float asNum;
  } floatPointNum;
  int xSpeed;
  int ySpeed;
  int xNumSteps;
  int yNumSteps;
  
  // Initialize accelerations to 0
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

  // Current time
  float currentTime;
  
  // Constants
  float timestep = 2.5004e-4;
  float ratio = 2.7929e-4; // Ratio of m/step, need to get real number and make sure its the same for both axis



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}



void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()>0){
    a4x = readInFloat();
    a4y = readInFloat();
    a4vx = readInFloat();
    a4vy = readInFloat();
    
    a3x = readInFloat();
    a3y = readInFloat();
    a3vx = readInFloat();
    a3vy = readInFloat();
    
    a2x = readInFloat();
    a2y = readInFloat();
    a2vx = readInFloat();
    a2vy = readInFloat();
     
    a1x = readInFloat();
    a1y = readInFloat();
    a1vx = readInFloat();
    a1vy = readInFloat();
    
    a0x = readInFloat();
    a0y = readInFloat();
    a0vx = readInFloat();
    a0vy = readInFloat();

    //Starts the interpolation here...
    currentTime = 0;
    
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
