/*
Fuel Injection System

Author: 
           * Damián Valderrama
           * Alejandro Pinto

INPUTS:
  - Intake Manifold Air Temperature
  - Intake Manifold Air Pressure
  - Engine RPM
  - Camshaft Position
OUTPUTS:
  - ti: Injector Duty Time
  - RPM Messure

PROCESSING EQUATIONS:
  - Gas Equation:
    P * V = n * R * T
      P: Absolute Intake Manifold Air Pressure [kg/(m*sˆ2)]
      V: Volume [mˆ3]
      n: Quantity of Gas [moles]
      R: Gas Constant [Joules] 
      T: Intake Manifold Air Temperature [K]
  - Air/Fuel Mixture:
    A = MAF/(RPM*N/2)
      N: #Cilinders
      MAF: n (Quantity of Gas [moles])
      RPM: Revolutions per Minute (Engine)
  - Fuel per Cilinder:
    F = A / (A/F)*d
      d: 
  - Injector Duty Time:
    ti = F / Rf
      Rf: Injector rate
*/

// Libraries

#include <LiquidCrystal.h>
const int rs = 7, en = 8, d4 = 9, d5 = 10, d6 = 11, d7 = 12;    //LCD interface pinout
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Setup Routine
void setup() 
{
  Serial.begin(9600);   // Initialize Serial Port - 9600 baudrate.
  lcd.begin(16, 2);     // set up the LCD's number of columns and rows:
  lcd.print ("Initialized");
  Serial.println("Initializing...");
}

// Main Routine
void loop()
{
}
