/*
Fuel Injection System

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

// Setup Routine
void setup() 
{
  Serial.begin(9600);    // Initialize Serial Port - 9600 baudrate.
}

// Main Routine
void loop()
{
Serial.println("Initializing...");
delay(1000);     // espera 1 segundo
}
