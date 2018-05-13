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
      P: Absolute Intake Manifold Air Pressure [kg/(m*sˆ2)] o [atm]
      V: Volume [mˆ3]
      n: Quantity of Gas [moles]
      R: Gas Constant [J / (mol·K)] 
      T: Intake Manifold Air Temperature [K]
  - Air/Fuel Mixture (A = n):
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

// Setup Variables
const int rs = 7, en = 8, d4 = 9, d5 = 10, d6 = 11, d7 = 12;    //LCD interface pinout
int analog_temp = 0;                                            // A0 analog input for TEMP sensor

// Global Variables
int temp = 273;                                                 // Default Temperature: 273K = 0C
int N = 4;                                                      // Number of Cilinders
int P = 1;                                                      // Default Atmospheric Pressure
float R = 0.18;                                                      // Default Gasoline Molecular Weight
int V = 1;                                                      // Default Air Mass Volume

// Initialization
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                      // LCD setup

// Functions

float A_calc (int P, int V, int T)
{
  float result;
  result = ((P * V) / (R * T));
  return result;
}

// Setup Routine
void setup() 
{
  Serial.begin(9600);                                          // Initialize Serial Port - 9600 baudrate.
  lcd.begin(16, 2);                                            // set up the LCD's number of columns and rows:
  lcd.print ("Initialized");
  Serial.println("Initializing...");
}

// Main Routine
void loop()
{
  temp = analogRead(analog_temp);                              // Read TEMP from A0 pin
  float MAF=(float)A_calc(P, V, temp);                         // Calculate Air/Fuel Mixture (n)
  lcd.clear();
  lcd.print(MAF,4);
  Serial.println(MAF,4);
}
