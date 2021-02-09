int dataCount = 0;
float data[5];

// The setup routine runs once when you press reset:
void setup() {
  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  while (!Serial){
    ; //wait for serial to connect
  }
  Serial.write("Serial Connected");
  Serial.write(13); //lf and cr
  Serial.write(10);
}


// The loop routine runs over and over again forever:
void loop() {
  // Write the sinewave points, followed by the terminator "Carriage Return" and "Linefeed".
  if(Serial.available() == 25){ //serial buffer has 5, 3 character floats in it, 10 bytes of lf/cr
    
    Serial.write("All Data Received");
    Serial.write(13); //lf and cr
    Serial.write(10);

    for (int i = 0; i<5; i++ ){ // loop through all values in serial buffer
      data[i] = Serial.parseFloat(SKIP_WHITESPACE); //will skip all white space
      // hopefully this only parses until the next whitespace, then removes that from the buffer
      // that way this method of looping through can work
      dataCount++; //increment datacount
    }

    for (int i = 0; i<5; i++){
      Serial.write(data[i]);//fix this, do some integer divisions then send each decimal at a time
      Serial.write(13);
      Serial.write(10);
    }

    Serial.write("All Data Processed and Stored");
    Serial.write(13); //lf and cr
    Serial.write(10);
  }

}
