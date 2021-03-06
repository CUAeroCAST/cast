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

bool moving = false;

int xDriverPUL = 24; //A2
int xDriverDIR = 26; //A4

int yDriverPUL = 9; //H6
int yDriverDIR = 8; //H5

int xlswitch0 = 50;
int ylswitch0 = 52;

bool xDirection = true; //true indicates positive direction movement
bool yDirection = true;

bool xPulseFlag = false;
bool yPulseFlag = false;

float xPos;
float yPos;
unsigned int xpulseDelay;
unsigned int ypulseDelay;
boolean toggleX = 0;
boolean toggleY = 0;

int xcounter = 0;
int ycounter = 0;
long xcounterPulse = 0;
long ycounterPulse = 0;

int pulseDelay = 225;

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

float absBounds = 0.25;

long xCounterBound = 3546;
long yCounterBound = 3546;

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
   OCR1A = 1000;//1000;// = (16*10^6) / (1*1024) - 1 (must be <65536)
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
   if(moving == false){
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
      //xCounterBound = ((int)(14184.4 * xStop));
      xCounterBound = ((long)(45390.5 * xStop));
      
      if (yStop>absBounds){
        yStop = absBounds;
      }
        //yCounterBound = ((int)(14184.4 *yStop));
      yCounterBound = ((long)(45390.5 *yStop));
      
      if(vxa0==0 && vxa1==0 && vya0==0 && vya1==0 && xStop==0 && yStop==0){//GO BACK TO CENTER
        restartTest();
      }else{// START TESTING
        moving = true;
        startTime = micros(); //Time that maneuver starts
        TIMSK1 |= (1 << OCIE1A); // enable interrupt for thrust curve
      }
    }
   }

   /////////////// Pulse x and y ////////////////
   //Change it so ycount and xcount increment no matter what, convert to position based on direction
  if(yPulseFlag==true){
    PORTH ^= _BV (6);
    yPulseFlag = false;
    ycounterPulse++;
  }
  if(xPulseFlag==true){
    PORTA ^= _BV (2);
    xPulseFlag = false;
    xcounterPulse++;
  }
  ////////////// Position Check ////////////////////
   //if (abs(xcounter)>xCounterBound | abs(ycounter)>yCounterBound){ //Stop gantry if position is +- .4 meters from center
   if (abs(xcounterPulse)>xCounterBound | abs(ycounterPulse)>yCounterBound){ //Stop gantry if position is +- .4 meters from center
    cli(); //disable interrupts    
    TIMSK1 = 0x00;
    TIMSK3 = 0x00; // turn off interrupt
    TIMSK4 = 0x00; // turn off interrupt
    cli();           // disable all interrupts
    moving = false;
    xcounter = xcounterPulse*.3125;
    ycounter = ycounterPulse*.3125;
    if(!xDirection){
      xcounter = -1*xcounter;
    }
    if(!yDirection){
      ycounter = -1*ycounter;
    }
    xPos = float(xcounter)*0.0000705;
    yPos = float(ycounter)*0.0000705;
    sei();
   }

}


///////////////////// ISR Timer 3 /////////////////////////////
// Toggle the x pulse pin //
ISR(TIMER3_COMPA_vect){ //change to timer3 when done testing
  xPulseFlag = true;
}

///////////////////// ISR Timer 4 /////////////////////////////
// Toggle the y pulse pin //
ISR(TIMER4_COMPA_vect){ //change to timer4 when done testing
  yPulseFlag = true;
}


/////////////////// ISR Timer 1 ////////////////////////////
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
  xpulseDelay = find_speedX(vxa0,vxa1,currentTime);
  ypulseDelay = find_speedY(vya0,vya1,currentTime);


  OCR3A = xpulseDelay - 1;
  OCR4A = ypulseDelay - 1;

  // encoder output
  xcounter = (int)(((float)xcounterPulse)*.3125);
  ycounter = (int)(((float)ycounterPulse)*.3125);
  if(!xDirection){
    xcounter = -xcounter;
  }
  if(!yDirection){
    ycounter = -ycounter;
  }
  Serial.write((xcounter & 0xFF));
  Serial.write((xcounter>>8 & 0xFF));

  Serial.write((ycounter & 0xFF));
  Serial.write((ycounter>>8 & 0xFF));

  TIMSK3 |= (1 << OCIE3A);
  TIMSK4 |= (1 << OCIE4A);
  sei();//allow interrupts
}

unsigned int find_speedY(float a0,float a1,float currentTime){
  float vel = a1*currentTime+a0;
  // change direction based on result
  if((vel>0)!=yDirection){
    //flip direction pin, if needed add delay
    PORTH ^= _BV (5);
    yDirection = !yDirection;
  }
  unsigned int pulseDelay = (unsigned int)(348.16/(abs(vel))); //1.36*16 is based on empirical calculation relating motor pulses to distance traveled
  if(abs(vel)<.0054){
    pulseDelay = 65000;
  }
  if(abs(vel)>.87){
    pulseDelay = 400;
  }
  return pulseDelay;
}

unsigned int find_speedX(float a0,float a1,float currentTime){
  float vel = a1*currentTime+a0;
  // change direction based on result
  if((vel>0)!=xDirection){
    //flip direction pin, if needed add delay
    PORTA ^= _BV (4);
    xDirection = !xDirection;
  }
  unsigned int pulseDelay = (unsigned int)(348.16/(abs(vel))); //1.36*16 is based on empirical calculation relating motor pulses to distance traveled
  if(abs(vel)<.0054){
    pulseDelay = 65000;
  }
  if(abs(vel)>.87){
    pulseDelay = 400;
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
   /////////////// Pulse x and y ////////////////
  if(xPulseFlag){
    PORTA ^= _BV (2);
    xPulseFlag = false;
  }
  if(yPulseFlag){
    PORTH ^= _BV (6);
    yPulseFlag = false;
  }
  // CHECK IF MANEUVER OUTSIDE OF RANGE, IF SO, TURN OFF TIMERS
  if(yPos>=.5){
      TIMSK4 &= (0 << OCIE4A); // turn off interrupt
    }
  if(xPos>=.5){
      TIMSK3 &= (0 << OCIE3A);
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
  xcounterPulse = 0;
  ycounterPulse = 0;
}

void restartTest(){
  // Flip directions
  cli();
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

  int xScale = 1;
  if(xPos<0){
    xScale = -1;
  }

  int yScale = 1; 
  if(yPos<0){
    yScale = -1;
  }
 
  while(xPos*xScale>0 || yPos*yScale>0){
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
   /////////////// Pulse x and y ////////////////
  if(xPulseFlag){
    PORTA ^= _BV (2);
    xPulseFlag = false;
  }
  if(yPulseFlag){
    PORTH ^= _BV (6);
    yPulseFlag = false;
  }
  // CHECK IF MANEUVER OUTSIDE OF RANGE, IF SO, TURN OFF TIMERS
  if(yPos*yScale<=0){
      TIMSK4 &= (0 << OCIE4A); // turn off interrupt
      //TIMSK4 = 0x00; // turn off interrupt
    }
  if(xPos*xScale<=0){
      TIMSK3 &= (0 << OCIE3A);
      //TIMSK3 = 0x00; // turn off interrupt
    }
  }
  TIMSK3 = 0x00; // turn off interrupt
  TIMSK4 = 0x00; // turn off interrupt
  cli();           // disable all interrupts
  sei();
}

float readInFloat(){
  for(int i=0;i<4;i++){
     floatPointNum.asBytes[i] = Serial.read();
   }
  //Serial.write(floatPointNum.asBytes,4);
  return floatPointNum.asNum;
}
