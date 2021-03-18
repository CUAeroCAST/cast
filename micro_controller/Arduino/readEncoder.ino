#include <math.h>

int xEA_plus = 32;
int xEB_plus = 30;

int yEA_plus = 3;
int yEB_plus = 5;

float xPos = 0;
float yPos = 0;

int xaState;
int xbState;
int xaLastState;
int yaState;
int yaLastState;
int ybState;

int xcounter = 0;
int ycounter = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(xEA_plus, INPUT);
  pinMode(xEB_plus, INPUT);
  pinMode(yEA_plus, INPUT);
  pinMode(yEB_plus, INPUT);
  xaLastState = digitalRead(xEA_plus);
  yaLastState = digitalRead(yEA_plus);


    // initialize timer1
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 57723;            // preload timer 65536-16MHz/256/4Hz
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}

ISR(TIMER1_OVF_vect){
  //Serial.write("Position X: ");
  TCNT1 = 49911;            // preload time
  Serial.print(xPos);
  //Serial.write("\n");
  //Serial.write("Position Y: ");
  Serial.write(",");
  Serial.print(yPos);
  Serial.write("\n");

}

void loop() {
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
}
