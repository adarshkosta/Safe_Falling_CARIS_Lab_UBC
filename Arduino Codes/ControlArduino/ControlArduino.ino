#include <SoftwareSerial.h>   //See limitations of Arduino SoftwareSerial

SoftwareSerial mySerial(10,11);

int th1=0;
byte valA1=0;
byte valB1=0;
byte valA2=0;
byte valB2=0;
byte code = 0;
byte b = 0xFF;
volatile int index = 0;
volatile bool dataStart = false, dataEnd = false;
volatile int data[2] = {64, 191};

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

  if(dataEnd == true && dataStart == false)
  {
     mySerial.write(data[0]);
     mySerial.write(data[1]);
  }
  
  delayMicroseconds(10);
 // delay(100);
}

void serialEvent() 
{
  while (Serial.available()) 
  {
    // get the new byte:
    int inData = (int)Serial.read();

    if (inData == 255) 
    {
      dataEnd = true;
      dataStart = false;
    }
    
    if(dataStart == true && dataEnd == false)
    {
       data[index] = inData;
       index++;
    }

    if (inData == 0) 
    {
      dataStart = true;
      dataEnd = false;
      index = 0;
    }
    
  }
}
