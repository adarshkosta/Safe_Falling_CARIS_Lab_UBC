
#include <avr/io.h>
#include <avr/interrupt.h>

#define enc0A 2
#define enc0B 4
#define enc1A 3
#define enc1B 5
#define POT_PIN A5

volatile int in[2] = {0};
volatile double theta[2] = {0};
volatile int int_theta[2] = {0};
volatile int potAdc = 0, ankleAngle = 0;

void setup() 
{
  EIMSK |= (1<<INT0)|(1<<INT1);
  EICRA |= (1<<ISC00)|(1<<ISC10);
  sei();

  pinMode(enc0A, INPUT); 
  pinMode(enc0B, INPUT); 
  pinMode(enc1A, INPUT); 
  pinMode(enc1B, INPUT); 
  
  Serial.begin (38400);
}

void getAnkleAngle()
{
   potAdc = analogRead(POT_PIN);
   ankleAngle = constrain(potAdc, 105, 771);
   ankleAngle = map(ankleAngle, 105, 771, 172, -7);   // Needs to be calibrated
}

void getJointAngles()
{
  getAnkleAngle();
  int_theta[0] = (int)theta[0];
  int_theta[1] = (int)theta[1];
}

void sendJointAngles()
{
  Serial.print(ankleAngle);
  Serial.print(",");
  Serial.print(theta[0]);
  Serial.print(",");
  Serial.print(theta[1]);
  Serial.println(",");
}

void loop() 
{
  getJointAngles();
  sendJointAngles();
  delayMicroseconds(200);
}


ISR(INT0_vect)
{
  in[0] = digitalRead(enc0B);
  
  if (digitalRead(enc0A) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (in[0] == 0) {  
      theta[0] = theta[0] + 0.234;         // CW
    } 
    else {
      theta[0] = theta[0] - 0.234;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (in[0] == 1) {   
      theta[0] = theta[0] + 0.234;          // CW
    } 
    else {
      theta[0] = theta[0] - 0.234;          // CCW
    }
  } 
}

ISR(INT1_vect)
{
  in[1] = digitalRead(enc1B);
  
  if (digitalRead(enc1A) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (in[1] == 0) 
    {  
      theta[1] = theta[1] + 0.234;         // CW
    } 
    else 
    {
      theta[1] = theta[1] - 0.234;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (in[1] == 1) 
    {   
      theta[1] = theta[1] + 0.234;          // CW
    } 
    else 
    {
      theta[1] = theta[1] - 0.234;          // CCW
    }
  } 
}
