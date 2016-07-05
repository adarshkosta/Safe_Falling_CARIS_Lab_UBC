int th1=0;
byte valA1=0;
byte valB1=0;
byte valA2=0;
byte valB2=0;
byte code = 0;
byte b = 0xFF;

void setup() {
  // initialize the serial communication:
  Serial.begin(38400);
  digitalWrite(3, HIGH);
  digitalWrite(2, HIGH);
  pinMode(2, INPUT);
  pinMode(4, INPUT);
  pinMode(3, INPUT);
  pinMode(5, INPUT);
}

void loop() {
  // send the value of analog input 0:
  th1=analogRead(A5);
  valA1=digitalRead(3) << 3;
  valB1=digitalRead(5) << 2;
  valA2=digitalRead(2) << 1; 
  valB2=digitalRead(4);
  code  = valA1 | valB1 | valA2 | valB2;

  if (th1 == 222) {
    th1 = 223;
  }

  
  Serial.write(th1/4);
  Serial.write(code);
  Serial.write(222);
 // Serial.println("");

  //Serial.write(b);
  //Serial.println(String(th1) + "," + String(valA1) + "," + String(valB1) + "," + String(valA2) + "," + String(valB2));
//  Serial.println("Start");
//  Serial.println(String(th1) + "test");
//  Serial.println("End");
  // wait a bit for the analog-to-digital c onverter
  // to stabilize after the last reading:
  
  delayMicroseconds(10);

 // delay(100);
}
