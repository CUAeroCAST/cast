byte tempReadIn[4];
union {
    byte asBytes[4];
    float asNum;
 } floatPointNum;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  int xDriverPUL = 7;
  int xDriverDIR = 6;
  int yDriverPUL = 5;
  int yDriverDIR = 4;
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Enter X distance in mm:\n");
  int xDist;
  int xVel;
  int yDist;
  int yVel;
  if (Serial.available()>0){
    int xDist = readInFloat();
    Serial.println("Enter X velocity in mm/s :\n");
    int xVel = readInFloat();
    Serial.println("Enter Y distance in mm:\n");
    int yDist = readInFloat();
    Serial.println("Enter Y velocity in mm/s :\n");
    int yVel = readInFloat();
  }
  int xSteps = find_steps(xDist);
  int ySteps = find_steps(yDist);
  int xSpeed = find_speed(xVel);
  int ySpeed = find_speed(yVel);
  //command_motor(xNumSteps,xSpeed,xDriverDIR,xDriverPUL);
  //command_motor(yNumSteps,ySpeed,yDriverDIR,yDriverPUL);
  

}

float readInFloat(){
  for(int i=0;i<4;i++){
     floatPointNum.asBytes[i] = Serial.read();
   }
  return floatPointNum.asNum;
}

int find_steps(int dist){
  float ratio = 2.7929e-4;
  int numSteps = round(dist*1000/ratio);
  return numSteps;
}

int find_speed(int vel){
  float ratio = 2.7929e-4;
  int stepSpeed = round(vel*1000/ratio);
  return stepSpeed;
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
