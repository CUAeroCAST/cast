int xEA_minus = 9;
int xEA_plus = 8;
int xEB_minus = 7;
int xEB_plus = 6;

int yEA_minus = 2;
int yEA_plus = 3;
int yEB_minus = 4;
int yEB_plus = 5;

int xaState;
int xaLastState;
int yaState;
int yaLastState;

int xDriverPUL = 10;
int xDriverDIR = 9;
int yDriverPUL = 10;
int yDriverDIR = 11;

int xlswitch0 = 51;
int ylswitch0 = 50;

float xPos;
float yPos;
int xpulseDelay;
int ypulseDelay;
int xcounter = 0;
int ycounter = 0;

bool testComplete = false;

float xa4 = -46.4830629783752;
float xa3 = 256.692660762877;
float xa2 = -530.959515178541;
float xa1 = 486.616060761667;
float xa0 = -166.533812886303;

float ya4 = 6.19682228638927;
float ya3 = -34.2215871056436;
float ya2 = 70.8022756878349;
float ya1 = -64.9124271668663;
float ya0 = 22.2246250399582;

float vxa4 = 8.73902649480158;
float vxa3 = -48.9521600754277;
float vxa2 = 102.734082857076;
float vxa1 = -95.7351150133154;
float vxa0 = 33.4139661600938;

float vya4 = -1.16048543803213;
float vya3 = 6.50166359753673;
float vya2 = -13.6471919249566;
float vya1 = 12.7198212155205;
float vya0 = -4.44043197560714;

float startTime = 1.199449908318053;
float currentTime = startTime;
float timestep = 0.000250041673612269;

int xSpeed;
int ySpeed;
int xNumSteps;
int yNumSteps;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  xSpeed = find_speed(vxa0,vxa1,vxa2,vxa3,vxa4,currentTime);
  ySpeed = find_speed(vya0,vya1,vya2,vya3,vya4,currentTime);
  xNumSteps = find_steps(xa0,xa1,xa2,xa3,xa4,currentTime);
  yNumSteps = find_steps(ya0,ya1,ya2,ya3,ya4,currentTime);
  command_motor(xNumSteps,xSpeed,xDriverDIR,xDriverPUL);
  command_motor(yNumSteps,ySpeed,yDriverDIR,yDriverPUL);
  currentTime = currentTime+timestep;
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
