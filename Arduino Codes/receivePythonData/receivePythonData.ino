#include <SoftwareSerial.h>   //See limitations of Arduino SoftwareSerial

SoftwareSerial mySerial(10,11);

String inputString = "";         // a string to hold incoming data
String str_pwmKnee = "", str_pwmHip = "";
boolean stringComplete = false;  // whether the string is complete
volatile int pwmKnee = 0, pwmHip = 0;

void setup() 
{

  Serial.begin(38400);
  mySerial.begin(38400);
  inputString.reserve(200);
}

void parse()
{
  int sLen = inputString.length();
  int i = 0;

  for(i=0; inputString[i] != ','; i++)
    str_pwmKnee += inputString[i];

  i++;

  for(;inputString[i] != '\n'; i++)
    str_pwmHip += inputString[i];

  pwmKnee = str_pwmKnee.toInt();
  pwmHip = str_pwmHip.toInt();
}

void loop() 
{
  if (stringComplete) 
  {
    parse();
//    if(pwmHip == 95 && pwmKnee == 40)
//      digitalWrite(13, HIGH);
//    else
//      digitalWrite(13, LOW); 

    mySerial.write(pwmKnee);
    mySerial.write(pwmHip);
    
    // clear the string:
    inputString = "";
    str_pwmKnee = "";
    str_pwmHip = "";
    stringComplete = false;
  }
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
