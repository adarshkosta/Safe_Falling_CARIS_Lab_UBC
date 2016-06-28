//Roboclaw simple serial example.  Set mode to 5.  Option to 4(38400 bps)
#include <SoftwareSerial.h>   //See limitations of Arduino SoftwareSerial
#include <avr/io.h>
#include <avr/interrupt.h>

#define enc0A 2
#define enc0B 4
#define enc1A 3
#define enc1B 5

#define S1 10
#define S2 11

#define POT_PIN A5
#define PID_LIMIT 1.0
#define PWM_LIMIT 63
#define ZERO_OFFSET 64
#define N 10
#define Kt 0.8576

volatile float torqueTarget[2] = {0.2, 0.2};

SoftwareSerial mySerial(10,11);

int CURR_SENSE_PIN_2 = A4;  // ACS714 +-30A
int CURR_SENSE_PIN_1 = A3;  // ACS711EX +-15.5A

const float kp[2] = {1.2, 1.2};
const float ki[2] = {0.0, 0.2};
const float kd[2] = {0.0, 0.0};

int initFlag = 0, index = 0;
volatile double sum[2] = {0}, currStck1[N] = {0}, currStck2[N] = {0};

volatile int potAdc;
volatile double curr[2] = {0};
volatile float error[2] = {0} , le[2] = {0};
volatile double t = 0 , lt = 0 ,dt = 0.01 ;
volatile float integral[2] = {0};
volatile float diff[2] = {0};
volatile float pid[2] = {0};
volatile double tau[2] = {0};
volatile double theta[2] = {0};
volatile double ankleAngle = 0;
volatile int in[2];

volatile int pwm[2];

volatile int runTime = 0, startTime = 0;
volatile int flag = 0;

void PID()
{
  int i;
  for(i=0; i<2; i++)
  {
    le[i] = error[i];
    error[i] = torqueTarget[i] - Kt*curr[i];
    integral[i] = integral[i] + error[i] * dt;
    diff[i] = (error[i] - le[i])/dt;
    
    pid[i] = kp[i]*error[i] +ki[i]*integral[i] + kd[i]*diff[i];
    pid[i] = constrain(pid[i], -PID_LIMIT, PID_LIMIT);
    
    pwm[i] = pid[i]*PWM_LIMIT/PID_LIMIT;
    pwm[i] = (int)(constrain(pwm[i],-PWM_LIMIT, PWM_LIMIT));
    
    if (abs(pwm[i])<5)
    {
      pwm[i] = 0;
    } 
  }  
  pwm[0] = pwm[0]*(-1);
  pwm[0] = pwm[0] + ZERO_OFFSET;  
  pwm[1] = pwm[1] - ZERO_OFFSET;  
}

void setup() 
{
  EIMSK |= (1<<INT0)|(1<<INT1);
  EICRA |= (1<<ISC00)|(1<<ISC10);
  EIMSK &= ~(1<<INT1);
  sei();
  
  pinMode(S1,OUTPUT);
  pinMode(S2,OUTPUT);

  pinMode(enc0A, INPUT); 
  pinMode(enc0B, INPUT); 
  pinMode(enc1A, INPUT); 
  pinMode(enc1B, INPUT); 
  
  Serial.begin (115200);
  mySerial.begin(38400);

  startTime = millis();
}

void getAnkleAngle()
{
   potAdc = analogRead(POT_PIN);
   ankleAngle = constrain(potAdc, 105, 771);
   ankleAngle = map(ankleAngle, 105, 771, 172, -7);   // Needs to be calibrated
}

// Print decimal numbers

void printDouble(double val, byte precision) 
{
  Serial.print (int(val));                                     // Print int part
  if( precision > 0) {                                         // Print decimal part
    Serial.print(".");
    unsigned long frac, mult = 1;
    byte padding = precision -1;
    while(precision--) mult *=10;
    if(val >= 0) frac = (val - int(val)) * mult; else frac = (int(val) - val) * mult;
    unsigned long frac1 = frac;
    while(frac1 /= 10) padding--;
    while(padding--) Serial.print("0");
    Serial.print(frac,DEC) ;
  }
}

double readInternalVcc()
{
  long result;
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);                                                    // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);                                         // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  double result1 = 1126400L / double(result);                                  // Back-calculate AVcc in mV
  return result1;
}

double currentSensor(int RawADC, int Sensitivity) 
{
  double InternalVcc    = readInternalVcc();
  double ZeroCurrentVcc = InternalVcc / 2.0;
  double SensedVoltage  = (double(RawADC) * double(InternalVcc)) / 1024.0;
  double Difference     = SensedVoltage - ZeroCurrentVcc;
  double SensedCurrent  = Difference / double(Sensitivity);
//  Serial.print("ADC: ");
//  Serial.print(RawADC);
//  Serial.print("/1024");
//  Serial.print(", Sensed Voltage: ");
//  printDouble(SensedVoltage, 1);
//  Serial.print("mV");
//  Serial.print(", 0A at: ");
//  printDouble(ZeroCurrentVcc, 1);
//  Serial.print("mV");
  return SensedCurrent;                                        // Return the Current
}

void getCurrents()
{
    sum[2] = {0};
    if(initFlag == 0)
    {
      for(int i=0; i<N; i++)
        {
          currStck1[i] = currentSensor(analogRead(CURR_SENSE_PIN_1), 132); 
          currStck2[i] = currentSensor(analogRead(CURR_SENSE_PIN_2), 66);
        }
        initFlag = 1;
    }
    else
    {
        if(index == N)
          index = 0;
        currStck1[index] = currentSensor(analogRead(CURR_SENSE_PIN_1), 132); 
        currStck2[index] = currentSensor(analogRead(CURR_SENSE_PIN_2), 66);
        index++;
    }
    sum[0] = 0;
    sum[1] = 0;
    for(int i=0; i<N; i++)
    {
       sum[0] += currStck1[i];
       sum[1] += currStck2[i];
    }

    curr[0] = (double)sum[0]/N;
    curr[1] = (double)sum[1]/N;
}

void _print()
{
    Serial.print("Curr[0]: ");
    Serial.print(curr[0]);
//    Serial.print("\tCurr2: ");
//    Serial.print(curr2);
//    Serial.print("\tPotAdc: ");
//    Serial.print(potAdc);
//    Serial.print ("\tAnkleAng:   ");
//    Serial.print (ankleAngle);
//    Serial.print ("\tKneeAng:   ");
//    Serial.print (theta[0]);
    Serial.print("\tPID: ");
    Serial.print (pid[0]);
    Serial.print("\tpwm0: ");
    Serial.print (pwm[0]);
    Serial.print(" \terr0: ");
    Serial.println(error[0]);
//    Serial.print(" \tintegral0: ");
//    Serial.print(integral[0]);
//    Serial.print(" \tdiff0: ");
//    Serial.println(diff[0]);
//    Serial.print ("\tHipAng:   ");
//    Serial.print (theta[1]);
//    //Serial.print("\tPID: ");
//    //Serial.print (pid);
//    Serial.print("\tpwm1: ");
//    Serial.print (pwm[1]);
//    Serial.print("\terr1: ");
//    Serial.print(error[1]);
//    
//    Serial.print("\tdt: ");
//    Serial.println(dt);

}

void _print1()
{
    if(pwm[0] != 64)
    {
      Serial.print("PotAdc: ");
      Serial.print(potAdc);
      Serial.print ("\tAnkleAng:   ");
      Serial.print (ankleAngle);
      Serial.print ("\tKneeAng:   ");
      Serial.print (theta[0]);
      //Serial.print("\tPID: ");
      //Serial.print (pid);
      Serial.print("\tpwm0: ");
      Serial.print (pwm[0]);
      Serial.print(" \terr0: ");
      Serial.print(error[0]);
      Serial.print ("\tHipAng:   ");
      Serial.print (theta[1]);
      //Serial.print("\tPID: ");
      //Serial.print (pid);
      Serial.print("\tpwm1: ");
      Serial.print (pwm[1]);
      Serial.print("\terr1: ");
      Serial.println(error[1]);
      flag = 1;
    }
    else
      {
        if(flag == 1)
        {
          Serial.print("PotAdc: ");
          Serial.print(potAdc);
          Serial.print ("\tAnkleAng:   ");
          Serial.print (ankleAngle);
          Serial.print ("\tKneeAng:   ");
          Serial.print (theta[0]);
          //Serial.print("\tPID: ");
          //Serial.print (pid);
          Serial.print("\tpwm0: ");
          Serial.print (pwm[0]);
          Serial.print(" \terr0: ");
          Serial.print(error[0]);
          Serial.print ("\tHipAng:   ");
          Serial.print (theta[1]);
          //Serial.print("\tPID: ");
          //Serial.print (pid);
          Serial.print("\tpwm1: ");
          Serial.print (pwm[1]);
          Serial.print("\terr1: ");
          Serial.println(error[1]);
          
          runTime = millis()-startTime;
          Serial.print("\nTime taken: ");
          Serial.print(runTime);
          Serial.println(" ms");
          flag = 0;
        }
      }
}

void actuate()
{
    mySerial.write(pwm[0]);
   // mySerial.write(pwm[1]);
}

void loop() 
{  
   // t = millis()/1000.0;
    getAnkleAngle();
    getCurrents();
    PID();   
    actuate();
    _print();
    //Serial.println(curr2);    
//    lt = t;
//    t = millis()/1000.0;
//    dt = t - lt;
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



