int xEA_minus = 5;
int xEA_plus = 2;
int xEB_minus = 3;
int xEB_plus = 4;

int xaState;
int xaLastState;

int xDriverPUL = 10;
int xDriverDIR = 9;
int yDriverPUL = 7;
int yDriverDIR = 6;

int xlswitch0 = 51;
//int ylswitch0 = 50;

float xPos;
float yPos;
int pulseDelay = 200;
int xcounter = 0;
int ycounter = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(xDriverPUL, OUTPUT);
  pinMode(xDriverDIR, OUTPUT);
  pinMode(yDriverPUL, OUTPUT);
  pinMode(yDriverDIR, OUTPUT);
  
  pinMode(xlswitch0, INPUT);
  //pinMode(ylswitch0, INPUT);

  pinMode(xEA_minus, INPUT);
  pinMode(xEA_plus, INPUT);
  pinMode(xEB_minus, INPUT);
  pinMode(xEB_plus, INPUT);
  // xaLastState = digitalRead(xEA_plus);
   // initialize timer1
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 4000;            // preload timer 65536-16MHz/256/2Hz
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  //interrupts();             // enable all interrupts


  homing();
  testRange();
}


ISR(TIMER1_OVF_vect){
  TCNT1 = 4000;            // preload time
  Serial.write("Position: ");
  Serial.print(xPos);
  Serial.write("\n");  
}

void loop() {

}



void homing(){
    // put your main code here, to run repeatedly:
  int xlimitState = digitalRead(xlswitch0);
  //int ylimitState = digitalRead(xlswitch0);
  digitalWrite(xDriverDIR,LOW);
  while(digitalRead(xlswitch0)){
      digitalWrite(xDriverPUL,HIGH);
      delayMicroseconds(pulseDelay);
      digitalWrite(xDriverPUL,LOW);
      delayMicroseconds(pulseDelay);
//    xaState = digitalRead(xEA_plus); // Reads the "current" state of the outputA
//    int xbState = digitalRead(xEB_plus);
//    // If the previous and the current state of the outputA are different, that means a Pulse has occured
//    if (xaState != xaLastState){
//      // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
//      if (digitalRead(xEB_plus) != xaState) {
//        xcounter ++; //cw
//      } 
//      else{
//        xcounter --; //ccw
//      }
//      xPos = xcounter*0.00005586;
//    }
//    xaLastState = xaState; // Updates the previous state of the outputA with the current state
  }
  xPos = 0;
  yPos = 0;
  xcounter = 0;
  Serial.write("Homing Complete \n");
  interrupts();          

  delay(1000);
  }

void testRange(){
  digitalWrite(xDriverDIR,HIGH); //switch dir
  float ratio = 0.00000021819;
  while(xPos<.5){
    digitalWrite(xDriverPUL,HIGH);
    delayMicroseconds(pulseDelay);
    digitalWrite(xDriverPUL,LOW);
    delayMicroseconds(pulseDelay);
    //xPos = xPos+ratio;
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
  noInterrupts();           // disable all interrupts
  Serial.write("Range Testing Complete");
}
