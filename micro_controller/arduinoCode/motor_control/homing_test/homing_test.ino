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
int pulseDelay = 150;
int xcounter = 0;
int ycounter = 0;

bool testComplete = false;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(xDriverPUL, OUTPUT);
  pinMode(xDriverDIR, OUTPUT);
  pinMode(yDriverPUL, OUTPUT);
  pinMode(yDriverDIR, OUTPUT);
  
  pinMode(xlswitch0, INPUT);
  pinMode(ylswitch0, INPUT);

  pinMode(xEA_minus, INPUT);
  pinMode(xEA_plus, INPUT);
  pinMode(xEB_minus, INPUT);
  pinMode(xEB_plus, INPUT);

  pinMode(yEA_minus, INPUT);
  pinMode(yEA_plus, INPUT);
  pinMode(yEB_minus, INPUT);
  pinMode(yEB_plus, INPUT);
  xaLastState = digitalRead(xEA_plus);
  yaLastState = digitalRead(yEA_plus);
   // initialize timer1
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 4000;            // preload timer 65536-16MHz/256/2Hz
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  //interrupts();             // enable all interrupts
}


ISR(TIMER1_OVF_vect){
  TCNT1 = 4000;            // preload time
  Serial.write("Position: ");
  Serial.print(xPos);
  Serial.print(", ");
  Serial.print(yPos);
  Serial.write("\n");  
}

void loop() {
  if(!testComplete){
    homing();
    testRange();
    testComplete = true;
  }
  else{
    Serial.print("Test Complete");
  }
}



void homing(){
    // put your main code here, to run repeatedly:
  bool xHome = false;
  bool yHome = false;
  int xlimitState = digitalRead(xlswitch0);
  int ylimitState = digitalRead(ylswitch0);
  digitalWrite(xDriverDIR,LOW);
  digitalWrite(yDriverDIR,LOW);
  while(!xHome && !yHome){
    xlimitState = digitalRead(xlswitch0);
    if(xlimitState ==1){
      digitalWrite(xDriverPUL,HIGH);
      delayMicroseconds(pulseDelay);
      digitalWrite(xDriverPUL,LOW);
      delayMicroseconds(pulseDelay);
    }
     else{
       xHome = true;
     }

    ylimitState = digitalRead(ylswitch0);
    if(ylimitState ==1){
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
  Serial.write("Homing Complete \n");
  interrupts();
  delay(1000);
}

void testRange(){
  digitalWrite(xDriverDIR,HIGH); //switch dir
  digitalWrite(yDriverDIR,HIGH);
  float ratio = 0.00000021819;
  while(xPos<.89 && yPos<.89){
    if(xPos<.89){
      digitalWrite(xDriverPUL,HIGH);
      delayMicroseconds(pulseDelay);
      digitalWrite(xDriverPUL,LOW);
      delayMicroseconds(pulseDelay);
      xaState = digitalRead(xEA_plus); // Reads the "current" state of the outputA
      int xbState = digitalRead(xEB_plus);
      // If the previous and the current state of the outputA are different, that means a Pulse has occured
      if (xaState != xaLastState){
        // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
        if (digitalRead(xEB_plus) != xaState) {
          xcounter ++; //cw
        } 
        else{
          xcounter --; //ccw
        }
        xPos = (float)xcounter*0.00005586;
      }
      xaLastState = xaState; // Updates the previous state of the outputA with the current state
    }
  }
  if(yPos<.89){
      digitalWrite(yDriverPUL,HIGH);
      delayMicroseconds(pulseDelay);
      digitalWrite(yDriverPUL,LOW);
      delayMicroseconds(pulseDelay);
      yaState = digitalRead(yEA_plus); // Reads the "current" state of the outputA
      int ybState = digitalRead(yEB_plus);
      // If the previous and the current state of the outputA are different, that means a Pulse has occured
      if (yaState != yaLastState){
        // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
        if (digitalRead(yEB_plus) != yaState) {
          ycounter ++; //cw
        } 
        else{
          ycounter --; //ccw
        }
        yPos = (float)ycounter*0.00005586;
      }
      yaLastState = yaState; // Updates the previous state of the outputA with the current state
    }
  noInterrupts();           // disable all interrupts
  Serial.write("Range Testing Complete");
}
