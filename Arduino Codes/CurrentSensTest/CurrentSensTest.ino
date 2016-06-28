//Roboclaw simple serial example.  Set mode to 5.  Option to 4(38400 bps)
#include <SoftwareSerial.h>

#define CURR_SENSE_PIN_1 A4   // ACS714 +-30A
#define CURR_SENSE_PIN_2 A3   // ACS711EX +-15.5A

//See limitations of Arduino SoftwareSerial

SoftwareSerial mySerial(10,11);
int i = 0;

volatile int currAdc1, currAdc2;
volatile float curr1=0, curr2=0;

void setup() 
{
  Serial.begin(115200);
  mySerial.begin(38400);
  delay(10);

  //mySerial.write(-40);
}

void getCurrents()
{
    currAdc1 = analogRead(CURR_SENSE_PIN_1)-512; 
    currAdc2 = analogRead(CURR_SENSE_PIN_2)-512;

    curr1 = ((float(currAdc1)/1024)*5)/0.066; //ACS714 -> 66mV/A 
    curr2 = ((float(currAdc2)/1024)*5)/0.136; //ACS711 -> 136mV/A 

    Serial.print("CurrAdc1: ");
    Serial.print(currAdc1);
    Serial.print("\tCurrAdc2: ");
    Serial.print(currAdc2);
    Serial.print("\tCurr1: ");
    Serial.print(curr1);
    Serial.print("\tCurr2: ");
    Serial.println(curr2);
}


void loop()
{
  for(i=64; i<128; i++)
  {
    mySerial.write(i);
    mySerial.write(-i);
    getCurrents();
    delay(200);
  }
  for(i=127; i>0; i--)
  {
    mySerial.write(i);
    mySerial.write(-i);
    getCurrents();
    delay(200);
  }
  for(i=1; i<65; i++)
  {
    mySerial.write(i);
    mySerial.write(-i);
    getCurrents();
    delay(200);
  }
  
}
