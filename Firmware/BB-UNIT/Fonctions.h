#include <Arduino.h>

// VOID FOR TAKE THE CURRENT
int current(int Cpin)
{
  int mVperAmp = 185; // use 100 for 20A Module and 66 for 30A Module
  int RawValue = 0;
  int ACSoffset = 2500;
  double Voltage = 0;
  double Amps = 0;

  RawValue = analogRead(Cpin);
  Voltage = (RawValue / 1024.0) * 5000; // Gets you mV
  Amps = ((Voltage - ACSoffset) / mVperAmp);

  return(Amps);
}

// VOID FOR TAKE THE BATTERY VOLTAGE
int voltage(int Vpin)
{
  float vout = 0.0;
  float vin = 0.0;
  float R1 = 100000.0; // resistance of R1 (100K) -see text!
  float R2 = 10000.0; // resistance of R2 (10K) - see text!
  int value = 0;

  value = analogRead(Vpin);
  vout = (value * 5.0) / 1024.0;
  vin = vout / (R2 / (R1 + R2));
  if (vin < 0.09) vin = 0.0;

  return(vin);
}

double Thermistor(int RawADC) {
 double Temp;
 Temp = log(10000.0*((1024.0/RawADC-1))); 
//         =log(10000.0/(1024.0/RawADC-1)) // for pull-up configuration
 Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
 Temp = Temp - 273.15;            // Convert Kelvin to Celcius
 return Temp;
}

void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
	asm volatile ("  jmp 0");  
}  
