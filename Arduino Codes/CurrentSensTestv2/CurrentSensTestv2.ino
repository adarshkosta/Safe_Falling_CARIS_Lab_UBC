int AnalogInputPin = A4; // Define analog input pin

void setup() 
{
  Serial.begin(115200);
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

long readInternalVcc()
{
  long result;
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);                                                    // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);                                         // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result;                                  // Back-calculate AVcc in mV
  return result;
}
double currentSensor(int RawADC) 
{
  int    Sensitivity    = 66; // mV/A
  long   InternalVcc    = readInternalVcc();
  double ZeroCurrentVcc = InternalVcc / 2;
  double SensedVoltage  = (RawADC * double(InternalVcc)) / 1024.0;
  double Difference     = SensedVoltage - ZeroCurrentVcc;
  double SensedCurrent  = Difference / Sensitivity;
  Serial.print("ADC: ");
  Serial.print(RawADC);
  Serial.print("/1024");
  Serial.print(", Sensed Voltage: ");
  printDouble(SensedVoltage, 1);
  Serial.print("mV");
  Serial.print(", 0A at: ");
  printDouble(ZeroCurrentVcc, 1);
  Serial.print("mV");
  return SensedCurrent;                                        // Return the Current
}

void loop() 
{
  double Current = currentSensor(analogRead(AnalogInputPin));  // Read  analog value
  Serial.print(", VariationFromNull: "); 
  printDouble(Current, 3);                                     // display Current
  Serial.print(" A");
  Serial.println("");
  delay(1000);

}
