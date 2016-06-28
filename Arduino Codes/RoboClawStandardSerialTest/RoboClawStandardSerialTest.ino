//Roboclaw simple serial example.  Set mode to 5.  Option to 4(38400 bps)
#include <SoftwareSerial.h>

//See limitations of Arduino SoftwareSerial

SoftwareSerial mySerial(10,11);
int i = 0;

void setup() 
{
  mySerial.begin(38400);
  delay(10);

 // mySerial.write(60);
}

void loop()
{
  for(i=64; i<128; i++)
  {
    mySerial.write(i);
    mySerial.write(-i);
    delay(100);
  }
  for(i=127; i>0; i--)
  {
    mySerial.write(i);
    mySerial.write(-i);
    delay(100);
  }
  for(i=1; i<65; i++)
  {
    mySerial.write(i);
    mySerial.write(-i);
    delay(100);
  }
  
}
