#define enc0A 2
#define enc0B 4
#define enc1A 3
#define enc1B 5

int th1=0;
byte val0A=0;
byte val0B=0;
byte val1A=0;
byte val1B=0;
byte code = 0;
byte b = 0xFF;

void setup() {
  // initialize the serial communication:
  Serial.begin(115200);
  pinMode(enc1A, INPUT);
  pinMode(enc1B, INPUT);
  pinMode(enc0A, INPUT);
  pinMode(enc0B, INPUT);
}

void loop() {
  // send the value of analog input 0:
  th1=analogRead(A5);
  val0A=digitalRead(enc0A) << 3;
  val0B=digitalRead(enc0B) << 2;
  val1A=digitalRead(enc1A) << 1; 
  val1B=digitalRead(enc1B);
  code  = val0A | val0B | val1A | val1B;

  if (th1 == 222) {
    th1 = 223;
  }

  
  Serial.print(int(th1/4));
  Serial.print(",");
  Serial.print(code);
  Serial.print(",");
  Serial.print(222);
  Serial.print(",");
  Serial.println(b);
//  Serial.println(String(th1) + "," + String(val0A) + "," + String(val0B) + "," + String(val1A) + "," + String(val1B));
//  Serial.println("Start");
//  Serial.println(String(th1) + "test"); 
//  Serial.println("End");
  // wait a bit for the analog-to-digital c onverter
  // to stabilize after the last reading:
  
  delay(1);

 // delay(100);
}
