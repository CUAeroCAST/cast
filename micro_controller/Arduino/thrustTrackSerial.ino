int xEA_plus = 32; //C5
int xEB_plus = 30; //C7

int yEA_plus = 3; //E5
int yEB_plus = 5; //E3

int xaState;
int xaLastState;
int yaState;
int yaLastState;
int xbState;
int ybState;

int xDriverPUL = 24; //A2
int xDriverDIR = 26; //A4

int yDriverPUL = 9; //H6
int yDriverDIR = 8; //H5

int xlswitch0 = 50;
int ylswitch0 = 52;

bool xDirection = true; //true indicates positive direction movement
bool yDirection = true;

float xPos;
float yPos;
long xpulseDelay;
long ypulseDelay;
boolean toggleX = 0;
boolean toggleY = 0;

int xcounter = 0;
int ycounter = 0;
int pulseDelay = 70;

bool testComplete = false;

float vxa4 = 0;
float vxa3 = 0;
float vxa2 = 0;
float vxa0 = 0;
float vxa1 = .002;

float vya4 = 0;
float vya3 = 0;
float vya2 = 0;
float vya0 = 0;
float vya1 = .001;

float xStop = 0.3;
float yStop = 0.3;

float absBounds = 0.4;

unsigned long startTime;
unsigned long currentMicros;

int xSpeed;
int ySpeed;
int xNumSteps;
int yNumSteps;


// Defining Variables
union {
    byte asBytes[4];
    float asNum;
} floatPointNum;



void setup() {
  // put your setup code here, to run once:
  cli();//stop interrupts
  Serial.begin(115200);
  Serial.flush();
  pinMode(xDriverPUL, OUTPUT);
  pinMode(yDriverPUL, OUTPUT);
  pinMode(xDriverDIR, OUTPUT);
  pinMode(yDriverDIR, OUTPUT);

  pinMode(xlswitch0, INPUT);
  pinMode(ylswitch0, INPUT);
  pinMode(xEA_plus, INPUT);
  pinMode(xEB_plus, INPUT);
  pinMode(yEA_plus, INPUT);
  pinMode(yEB_plus, INPUT);

  xaLastState = digitalRead(xEA_plus);
  yaLastState = digitalRead(yEA_plus);

  ////////////////// TIMER SETUPS ////////////////////////
  ////// TIMER3 Corresponds to x pulses ///////
  ////// TIMER4 Corresponds to y pulses ///////
  ////// TIMER5 Corresponds to output and pd adjustment ///////

   // SETUP TIMER TO TRIGGER EVERY 50ms, can adjust later //
   //set timer4 interrupt at 1Hz
   TCCR1A = 0;// set entire TCCR1A register to 0
   TCCR1B = 0;// same for TCCR1B
   TCNT1  = 0;//initialize counter value to 0
   // set compare match register for 1hz increments
   OCR1A = 2000;// = (16*10^6) / (1*1024) - 1 (must be <65536)
   // turn on CTC mode
   TCCR1B |= (1 << WGM12);
   // Set CS12 and CS10 bits for 1024 prescaler
   TCCR1B |= (1 << CS12) | (1 << CS10);
   // enable timer compare interrupt
   TIMSK1 |= (0 << OCIE1A);


  // SETUP TIMER TO WITH NO PRESCALE... keep disabled, will be enabled and set in TIMER5 ISR //
  TCCR3A = 0;// set entire TCCR2A register to 0
  TCCR3B = 0;// same for TCCR2B
  TCNT3  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  //OCR3A = 780;// = (16*10^6) / (8000*8) - 1 (must be <65536)
  // turn on CTC mode
  TCCR3B |= (1 << WGM32);
  // Set CS21 bit for no prescaler
  TCCR3B |= (1 << CS30);
  // enable timer compare interrupt
  //TIMSK3 |= (1 << OCIE5A);

  // SETUP TIMER TO WITH NO PRESCALE... keep disabled, will be enabled and set in TIMER5 ISR //
  TCCR4A = 0;// set entire TCCR2A register to 0
  TCCR4B = 0;// same for TCCR2B
  TCNT4  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  //OCR5A = 780;// = (16*10^6) / (8000*8) - 1 (must be <65536)
  // turn on CTC mode
  TCCR4B |= (1 << WGM42);
  // Set CS21 bit for 1024 prescaler
  TCCR4B |= (1 << CS40);
  // enable timer compare interrupt
  //TIMSK5 |= (1 << OCIE5A);

    //////////////////// Move to home then move to middle to start test /////////////////////
    homing();
    testRange();
    sei();
}

void loop() {
   if(Serial.available()==24){

    // READ IN MANUEVER COMMANDS
    vxa0 = readInFloat();
    vxa1 = readInFloat();
    vya0 = readInFloat();
    vya1 = readInFloat();
    xStop = readInFloat();
    yStop = readInFloat();

    // Adjust bounds to make sure they fall within the maximum
    if (xStop>absBounds){
      xStop = absBounds;
    }
    if (yStop>absBounds){
      yStop = absBounds;
    }   
    if(vxa0 && vxa1 && vya0 && vya1 && xStop && yStop){//GO BACK TO CENTER
      restartTest();
    }else{// START TESTING
      startTime = micros(); //Time that maneuver starts
      TIMSK1 |= (1 << OCIE1A); // enable interrupt for thrust curve
    }
   }


 ////////////////////////////// GET ENCODER FEEDBACK ////////////////////////////////////
   xaState = (PINC & _BV (5)) == 0; // digitalRead (8);
   xbState = (PINC & _BV (7)) == 0; // digitalRead (6);

   yaState = (PINE & _BV (5)) == 0; // digitalRead (3);
   ybState = (PINE & _BV (3)) == 0; // digitalRead (5);

   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (xaState != xaLastState){
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (xbState != xaState) {
       xcounter ++; //cw towards is positive
     } else {
       xcounter --; //ccw away is negative
     }
      xPos = float(xcounter)*0.0000705;
   }

   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (yaState != yaLastState){
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (ybState != yaState) {
       ycounter ++; //cw towards is positive
     } else {
       ycounter --; //ccw away is negative
     }
     yPos = float(ycounter)*0.0000704995;
   }
   xaLastState = xaState; // Updates the previous state of the outputA with the current state
   yaLastState = yaState; // Updates the previous state of the outputA with the current state
 /////////////////////////////////////////////////////////////////////////////////////////////

  ////////////// Position Check ////////////////////
   if (abs(xPos)>xStop | abs(yPos)>yStop){ //Stop gantry if position is +- .4 meters from center
    TIMSK5 |= (0 << OCIE5A);
    TIMSK3 |= (0 << OCIE3A);
    TIMSK4 |= (0 << OCIE4A);
    cli(); //disable interrupts
   }
}


///////////////////// ISR Timer 3 /////////////////////////////
// Toggle the x pulse pin //
ISR(TIMER3_COMPA_vect){ //change to timer3 when done testing
  PORTA ^= _BV (2);
}

///////////////////// ISR Timer 4 /////////////////////////////
// Toggle the y pulse pin //
ISR(TIMER4_COMPA_vect){ //change to timer4 when done testing
  PORTH ^= _BV (6);
}


/////////////////// ISR Timer 5 ////////////////////////////
// Disable timer interrupts //
// Compute new pulse delays //
// Set compare registers //
// Serial write encoder positions //
// Enable interrupts //
ISR(TIMER1_COMPA_vect){
  TIMSK3 |= (0 << OCIE3A);
  TIMSK4 |= (0 << OCIE4A);
  cli(); //disable interrupts

  currentMicros = micros();
  float currentTime = (float(currentMicros - startTime)) / float(pow(10,6)); //convert to seconds
  xpulseDelay = find_speedX(vxa0,vxa1,vxa2,vxa3,vxa4,currentTime);
  ypulseDelay = find_speedY(vya0,vya1,vya2,vya3,vya4,currentTime);


  OCR3A = xpulseDelay*16 - 1;
  OCR4A = ypulseDelay*16 - 1;

  // encoder output
  Serial.write((xcounter & 0xFF));
  Serial.write((xcounter>>8 & 0xFF));

  Serial.write((ycounter & 0xFF));
  Serial.write((ycounter>>8 & 0xFF));

  TIMSK3 |= (1 << OCIE3A);
  TIMSK4 |= (1 << OCIE4A);
  sei();//allow interrupts

}

long find_speedY(float a0,float a1,float a2,float a3,float a4,float currentTime){
  float vel = a4*pow(currentTime,4)+a3*pow(currentTime,3)+a2*pow(currentTime,2)+a1*currentTime+a0;
  // change direction based on result
  if((vel>0)!=yDirection){
    //flip direction pin, if needed add delay
    PORTH ^= _BV (5);
    yDirection = !yDirection;
  }
  long pulseDelay = (1.36/(abs(vel))); //1.36 is based on empirical calculation relating motor pulses to distance traveled
  if(pulseDelay>4096){
    pulseDelay = 4096;
  }
  return pulseDelay;
}

long find_speedX(float a0,float a1,float a2,float a3,float a4,float currentTime){
  float vel = a4*pow(currentTime,4)+a3*pow(currentTime,3)+a2*pow(currentTime,2)+a1*currentTime+a0;
  // change direction based on result
  if((vel>0)!=xDirection){
    //flip direction pin, if needed add delay
    PORTA ^= _BV (4);
    xDirection = !xDirection;
  }
  long pulseDelay = (1.36/(abs(vel))); //1.4959 is based on empirical calculation relating motor pulses to distance traveled
  if(pulseDelay>4096){
    pulseDelay = 4096;
  }
  return pulseDelay;
}

void homing(){
  // put your main code here, to run repeatedly:
  bool xHome = false;
  bool yHome = false;
  int xlimitState = digitalRead(xlswitch0);
  int ylimitState = digitalRead(ylswitch0);
  Serial.print(ylimitState);
  digitalWrite(xDriverDIR,HIGH);
  digitalWrite(yDriverDIR,HIGH);
  delayMicroseconds(pulseDelay*2);
  while(!xHome || !yHome){
    xlimitState = digitalRead(xlswitch0);
    if(xlimitState==1 && xHome == false){
      digitalWrite(xDriverPUL,HIGH);
      delayMicroseconds(pulseDelay);
      digitalWrite(xDriverPUL,LOW);
      delayMicroseconds(pulseDelay);
    }
     else{
       xHome = true;
     }

    ylimitState = digitalRead(ylswitch0);
    if(ylimitState==1 && yHome == false){
      digitalWrite(yDriverPUL,HIGH);
      delayMicroseconds(pulseDelay);
      digitalWrite(yDriverPUL,LOW);
      delayMicroseconds(pulseDelay);
    }
    else{
      yHome = true;
    }
  }
  xPos = 0;
  xcounter = 0;
  yPos = 0;
  ycounter = 0;
  delay(1000);
}

void testRange(){
  digitalWrite(xDriverDIR,LOW); //switch dir
  digitalWrite(yDriverDIR,LOW);
  delayMicroseconds(pulseDelay*2);

  OCR3A = pulseDelay*16 - 1;
  OCR4A = pulseDelay*16 - 1;
  TIMSK3 |= (1 << OCIE3A);
  TIMSK4 |= (1 << OCIE4A);
  sei();

  while(xPos<.5 || yPos<.5){
 ////////////////////////////// GET ENCODER FEEDBACK ////////////////////////////////////
//   Serial.print(xPos);
//   Serial.write("\n");
   xaState = (PINC & _BV (5)) == 0; // digitalRead (8);
   xbState = (PINC & _BV (7)) == 0; // digitalRead (6);

   yaState = (PINE & _BV (5)) == 0; // digitalRead (3);
   ybState = (PINE & _BV (3)) == 0; // digitalRead (5);

   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (xaState != xaLastState){
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (xbState != xaState) {
       xcounter ++; //cw towards is positive
     } else {
       xcounter --; //ccw away is negative
     }
      xPos = float(xcounter)*0.0000705;
   }

   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (yaState != yaLastState){
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (ybState != yaState) {
       ycounter ++; //cw towards is positive
     } else {
       ycounter --; //ccw away is negative
     }
     yPos = float(ycounter)*0.0000704995;
   }
   xaLastState = xaState; // Updates the previous state of the outputA with the current state
   yaLastState = yaState; // Updates the previous state of the outputA with the current state
 /////////////////////////////////////////////////////////////////////////////////////////////

  // CHECK IF MANEUVER OUTSIDE OF RANGE, IF SO, TURN OFF TIMERS
  if(yPos>=.5){
      TIMSK4 |= (0 << OCIE4A); // turn off interrupt
    }
  if(xPos>=.5){
      TIMSK3 |= (0 << OCIE3A);
    }
  }
  TIMSK3 = 0x00; // turn off interrupt
  TIMSK4 = 0x00; // turn off interrupt
  cli();           // disable all interrupts
  // New 0,0
  xPos = 0;
  yPos = 0;
  xcounter = 0;
  ycounter = 0;
}

void restartTest(){
  // Flip directions
  PORTH ^= _BV (5);
  yDirection = !yDirection;
  PORTA ^= _BV (4);
  xDirection = !xDirection;
  delayMicroseconds(pulseDelay*2);

  OCR3A = pulseDelay*16 - 1;
  OCR4A = pulseDelay*16 - 1;
  TIMSK3 |= (1 << OCIE3A);
  TIMSK4 |= (1 << OCIE4A);
  sei();
  
  while(xPos!=0 || yPos!=0){
 ////////////////////////////// GET ENCODER FEEDBACK ////////////////////////////////////
//   Serial.print(xPos);
//   Serial.write("\n");
   xaState = (PINC & _BV (5)) == 0; // digitalRead (8);
   xbState = (PINC & _BV (7)) == 0; // digitalRead (6);

   yaState = (PINE & _BV (5)) == 0; // digitalRead (3);
   ybState = (PINE & _BV (3)) == 0; // digitalRead (5);

   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (xaState != xaLastState){
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (xbState != xaState) {
       xcounter ++; //cw towards is positive
     } else {
       xcounter --; //ccw away is negative
     }
      xPos = float(xcounter)*0.0000705;
   }

   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (yaState != yaLastState){
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (ybState != yaState) {
       ycounter ++; //cw towards is positive
     } else {
       ycounter --; //ccw away is negative
     }
     yPos = float(ycounter)*0.0000704995;
   }
   xaLastState = xaState; // Updates the previous state of the outputA with the current state
   yaLastState = yaState; // Updates the previous state of the outputA with the current state
 /////////////////////////////////////////////////////////////////////////////////////////////

  // CHECK IF MANEUVER OUTSIDE OF RANGE, IF SO, TURN OFF TIMERS
  if(yPos!=0){
      TIMSK4 |= (0 << OCIE4A); // turn off interrupt
    }
  if(xPos!=0){
      TIMSK3 |= (0 << OCIE3A);
    }
  }
  TIMSK3 = 0x00; // turn off interrupt
  TIMSK4 = 0x00; // turn off interrupt
  cli();           // disable all interrupts
}

float readInFloat(){
  for(int i=0;i<4;i++){
     floatPointNum.asBytes[i] = Serial.read();
   }
  //Serial.write(floatPointNum.asBytes,4);
  return floatPointNum.asNum;
}
