//Roboclaw simple serial example.  Set mode to 5.  Option to 4(38400 bps)
#include <SoftwareSerial.h>   //See limitations of Arduino SoftwareSerial
#include <avr/io.h>

#define S1 10
#define S2 11
#define N 1

SoftwareSerial mySerial(10,11);

int CURR_SENSE_PIN_2 = A4;  // ACS714 +-30A
int CURR_SENSE_PIN_1 = A3;  // ACS711EX +-15.5A

int initFlag = 0, index = 0;
volatile double sum[2] = {0}, currStck1[N] = {0}, currStck2[N] = {0};

volatile double curr1, curr2;

volatile double start=0, t=0, maxA=0, minA=0, noise=0;

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
//  Serial.print("Internal VCC: ");
//  Serial.print(result1);
  return result1;
}

double currentSensor(int RawADC, int Sensitivity) 
{
//  double InternalVcc    = readInternalVcc();
//  double ZeroCurrentVcc = InternalVcc / 2.0;
//  double SensedVoltage  = (double(RawADC) * double(InternalVcc)) / 1024.0;
//  double Difference     = SensedVoltage - ZeroCurrentVcc;
//  double SensedCurrent  = Difference / double(Sensitivity);

//  Serial.print("ADC: ");
//  Serial.print(RawADC);
//  Serial.print("/1024");
//  Serial.print(", Sensed Voltage: ");
//  printDouble(SensedVoltage, 1);
//  Serial.print("mV");
//  Serial.print(", 0A at: ");
//  Serial.print(ZeroCurrentVcc);
//  Serial.print("mV");

  double SensedCurrent = (516-double(RawADC))*5000/1024.0/double(Sensitivity);

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

    curr1 = (double)sum[0]/N;
    curr2 = (double)sum[1]/N;
    t = millis() - start;
    t = t/1000.0;
}

void _print()
{
//    Serial.print("Curr1: ");
    Serial.print(t);
    Serial.print("\t");
    Serial.print(curr2);
    Serial.print("\t");
    Serial.print(noise);
    Serial.print("\t");
    Serial.print(maxA);
    Serial.print("\t");
    Serial.println(minA);
//    Serial.print("\tCurr2: ");
//    Serial.print(curr2);
}

void setup() 
{
  pinMode(S1,OUTPUT);
  pinMode(S2,OUTPUT);

  Serial.begin (115200);
  mySerial.begin(38400);

  delay(500);
  mySerial.write(-90);
  delay(1000);

  start = millis();

  
}

void loop() 
{  
  getCurrents();
  maxA = max(maxA, curr2);
  minA = min(minA, curr2);
  noise = maxA-minA;
  _print();
  delay(1);
}
