#include <SoftwareSerial.h>

SoftwareSerial mySerial(10,11);

void setup() {
Serial.begin(38400); // set the baud rate
Serial.println("Ready"); // print "Ready" once
mySerial.begin(38400);
mySerial.write(90);
}
void loop() {
char inByte = ' ';
if(Serial.available()){ // only send data back if data has been sent
char inByte = Serial.read(); // read the incoming data
mySerial.write((int)inByte);
Serial.println(inByte); // send the data back in a new line so that it is not all one long line. 
// for debugging only
}
delayMicroseconds(50); // delay 
}
