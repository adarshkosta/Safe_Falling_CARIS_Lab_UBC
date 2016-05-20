#include <avr/io.h>
#include <avr/interrupt.h>

#define enc0A 2
#define enc0B 4
#define enc1A 3
#define enc1B 5
#define S1 10
#define S2 11

#define POT_PIN A5
#define PID_LIMIT 220

const float kp[2] = {5.5, 5.5};
const float ki[2] = {0.0, 0.0};
const float kd[2] = {0.0, 0.0};

volatile int potAdc;
volatile float target[2] = {30, 30};
volatile float error[2] = {0} , le[2] = {0};
volatile double t = 0 , lt = 0 ,dt = 0 ;
volatile float integral[2] = {0};
volatile float diff[2] = {0};
volatile float pid[2] = {0};
volatile double theta[2] = {0};
volatile double ankleAngle = 0;
volatile int in[2];

volatile int pwm[2];

void PID()
{
  int i;
  for(i=0; i<2; i++)
  {
    le[i] = error[i];
    error[i] = target[i] - theta[i];
    integral[i] = integral[i] + error[i] * dt;
    diff[i] = (error[i] - le[i])/dt;
    pid[i] = kp[i]*error[i] +ki[i]*integral[i] + kd[i]*diff[i];
      
    pwm[i] = (int)(constrain(pid[i],-PID_LIMIT,PID_LIMIT));
    pwm[i] = pwm[i]*(-1);
   
    if (abs(pwm[i])<15)
    {
      pwm[i] = 0;
    }
  }  
}

void setup() 
{
  EIMSK |= (1<<INT0)|(1<<INT1);
  EICRA |= (1<<ISC00)|(1<<ISC10);
  sei();
  
  pinMode(S1,OUTPUT);
  pinMode(S2,OUTPUT);

  pinMode(enc0A, INPUT); 
  pinMode(enc0B, INPUT); 
  pinMode(enc1A, INPUT); 
  pinMode(enc1B, INPUT); 
  
  Serial.begin (115200);
}

void getAnkleAngle()
{
   potAdc = analogRead(POT_PIN);
   ankleAngle = constrain(potAdc, 105, 771);
   ankleAngle = map(ankleAngle, 105, 771, 172, -7);   // Needs to be calibrated
}

void _print()
{
    Serial.print("Pot_ADC: ");
    Serial.print(potAdc);
    Serial.print ("\tAnkle Angle:   ");
    Serial.print (ankleAngle);
    Serial.print ("\tKneeAngle:   ");
    Serial.print (theta[0]);
//    Serial.print("\tPID: ");
//    Serial.print (pid);
    Serial.print("\tpwm0: ");
    Serial.print (pwm[0]);
    Serial.print("\terr0: ");
    Serial.print(error[0]);
    Serial.print ("\tHipAngle:   ");
    Serial.print (theta[1]);
//    Serial.print("\tPID: ");
//    Serial.print (pid);
    Serial.print("\tpwm1: ");
    Serial.print (pwm[1]);
    Serial.print("\tErr1: ");
    Serial.println(error[1]);
}

void loop() 
{  
    t = millis()/1000.0;
    getAnkleAngle();
    PID();   
    //rotate(pwm);
    _print();    
    lt = t;
    t = millis()/1000.0;
    dt = t - lt;
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



