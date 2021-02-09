#include <math.h>
void setup() {
  // put your setup code here, to run once:
  int xDriverPUL = 7;
  int xDriverDIR = 6;
  int yDriverPUL = 5;
  int yDriverDIR = 4;
  float currentTime;
  float timeStart;
  float timestep = 2.5004e-4;
  float ratio = 0.001; // Ratio of m/step, need to get real number and make sure its the same for both axis
  Serial.begin(9600);
  if (Serial.available()>0){
    float a4x = Serial.parseFloat();
    float a4y = Serial.parseFloat();
    float a4vx = Serial.parseFloat();
    float a4vy = Serial.parseFloat();
    
    float a3x = Serial.parseFloat();
    float a3y = Serial.parseFloat();
    float a3vx = Serial.parseFloat();
    float a3vy = Serial.parseFloat();
    
    float a2x = Serial.parseFloat();
    float a2y = Serial.parseFloat();
    float a2vx = Serial.parseFloat();
    float a2vy = Serial.parseFloat();
     
    float a1x = Serial.parseFloat();
    float a1y = Serial.parseFloat();
    float a1vx = Serial.parseFloat();
    float a1vy = Serial.parseFloat();
    
    float a0x = Serial.parseFloat();
    float a0y = Serial.parseFloat();
    float a0vx = Serial.parseFloat();
    float a0vy = Serial.parseFloat();
    
    timeStart = Serial.parseFloat();
    currentTime = Serial.parseFloat();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()>0){
    a4x = Serial.parseFloat();
    a4y = Serial.parseFloat();
    a4vx = Serial.parseFloat();
    a4vy = Serial.parseFloat();
    
    a3x = Serial.parseFloat();
    a3y = Serial.parseFloat();
    a3vx = Serial.parseFloat();
    a3vy = Serial.parseFloat();
    
    a2x = Serial.parseFloat();
    a2y = Serial.parseFloat();
    a2vx = Serial.parseFloat();
    a2vy = Serial.parseFloat();
     
    a1x = Serial.parseFloat();
    a1y = Serial.parseFloat();
    a1vx = Serial.parseFloat();
    a1vy = Serial.parseFloat();
    
    a0x = Serial.parseFloat();
    a0y = Serial.parseFloat();
    a0vx = Serial.parseFloat();
    a0vy = Serial.parseFloat();
    
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

int find_speed(a0,a1,a2,a3,a4,currentTime){
  float vel = a4*currentTime^4+a3*currentTime^3+a2*currentTime^2+a1*currentTime^1+a0
  int stepSpeed = round(vel/ratio);
  return stepSpeed;
}

int find_steps(a0,a1,a2,a3,a4,currentTime){
  float dist = a4*currentTime^4+a3*currentTime^3+a2*currentTime^2+a1*currentTime^1+a0;
  int numSteps = round(dist/ratio);
  return numSteps;
}

void command_motor(numSteps,stepSpeed,driverDIR,driverPUL){
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
