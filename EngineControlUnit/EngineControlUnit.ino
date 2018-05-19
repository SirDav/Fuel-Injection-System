/*
Fuel Injection System

Author: 
           * Damián Valderrama
           * Alejandro Pinto

INPUTS:
  - Intake Manifold Air Temperature
  - Intake Manifold Air Pressure
  - Engine RPM
  
OUTPUTS:
  - ti: Injector Duty Time
  - RPM Measure

PROCESSING EQUATIONS:
  - Gas Equation:
    P * V = n * R * T
      P: Absolute Intake Manifold Air Pressure [kg/(m*sˆ2)] o [atm]
      V: Volume [mˆ3]
      n: Quantity of Gas [moles]
      R: Gas Constant [J / (mol·K)] 
      T: Intake Manifold Air Temperature [K]
  - Air/Fuel Mixture (A = n):
    A = MAF/((RPM/60)*N/2)
      N: #Cilinders
      MAF: n (Quantity of Gas [moles])
      RPM: Revolutions per Minute (Engine)
  - Fuel per Cilinder:
    F = A / AFR
      AFR: air-fuel ration
  - Injector Duty Time:
    ti = F / Rf
      Rf: Injector rate
*/

// Libraries
#include <LiquidCrystal.h>
#include <TimerOne.h>

// Defines
#define ADC_RESOL      10                                   // ADC resolution (number of bits of ADC) 
#define TEMP_RANGE     90                                   // Intake manifold air temperature range
#define TEMP_MIN      240                                   // Minimum intake manifold air temperature [°K]
#define TEMP_MAX      330                                   // Maximum intake manifold air temperatura [°K]
#define WAITING 0
#define INUSE 1

//VE Matrix
float VE_Matrix[9][18] = {
  {7.3, 55.0, 55.0, 57.0, 63.5, 67.0,  66.0, 74.0, 71.5, 75.7, 77.5, 75.5, 75.5, 75.5, 73.0, 72.0, 72.0, 72.0 },
  {9.2, 55.0, 55.0, 59.0, 65.0, 69.0,  68.0, 76.0, 73.5, 78.0, 79.5, 78.0, 78.0, 78.0, 75.5, 74.0, 74.0, 74.0 },
  {11.0, 55.0, 55.0, 62.5, 68.5, 73.5,  72.0, 80.0, 78.0, 82.0, 84.0, 82.0, 82.0, 82.0, 79.5, 78.5, 78.5, 78.5 },
  {12.9, 55.0, 56.5, 64.5, 71.5, 76.0,  74.5, 83.0, 80.5, 85.5, 87.5, 85.5, 85.5, 85.5, 82.5, 81.0, 81.0, 81.0 },
  {14.7, 57.5, 59.5, 68.0, 75.0, 80.0,  78.5, 88.0, 85.0, 90.0, 92.0, 90.0, 90.0, 90.0, 87.0, 85.5, 85.5, 85.5 },
  {16.5, 57.5, 59.5, 68.0, 75.0, 80.0,  78.5, 88.0, 85.0, 90.0, 92.0, 90.0, 90.0, 90.0, 87.0, 85.5, 85.5, 85.5 },
  {18.4, 59.0, 61.5, 69.5, 77.0, 82.0,  80.5, 90.0, 87.0, 92.5, 94.0, 92.5, 92.5, 92.5, 89.5, 88.0, 88.0, 88.0 },
  {20.2, 59.0, 61.5, 69.5, 77.0, 82.0,  80.5, 90.0, 87.0, 92.5, 94.0, 92.5, 92.5, 92.5, 89.5, 88.0, 88.0, 88.0 },
  {22.0, 59.0, 61.5, 69.5, 77.0, 82.0,  80.5, 90.0, 87.0, 92.5, 94.0, 92.5, 92.5, 92.5, 89.5, 88.0, 88.0, 88.0 }
};

// Setup Variables
const int rs = 7, en = 8, d4 = 9, d5 = 10, d6 = 11, d7 = 12;    //LCD interface pinout
int analog_temp = 0;                                            // A0 analog input for TEMP sensor
const int buttonPin = 2;                                        // External button for menu
const int ledPin =  13;                                         // the number of the LED pin

// Global Variables
int lastButtonState = LOW;
int buttonState;
unsigned display_refresh = 30000;                               // Display Refresh Rate [us]
unsigned refresh_counter = 0;
unsigned long lastDebounceTime = 0;                             // Last Time the Key was Toggled
unsigned long debounceDelay = 50;                               // Debounce Time; Increase if the Input Flickers
int display_message = 0;                                        // Display Screen in use (Value to Print)
float temp = 300.00;                                            // Default Temperature: 300°K = 26.85°C
int N = 4;                                                      // Number of Cilinders
float P = 1.0;                                                  // Default Atmospheric Pressure [atm]
float R = 0.000082057;                                          // Default Gasoline Molecular Weight [(m^3*atm)/(°K*mol)
float V = 1.00;                                                 // Default Air Mass Volume [m^3]
float MAF = 0;
int A = 0;
int RPM = 1000;                                               
int Inj_output = 3;                                             // Output pin for Injector
float Rf = 2.5;                                                   // Injector rate [g/s]
float AFR = 14.7;                                               // Stoichiometric air-fuel ratio. 
float engineDispl = 2.0;                                        // Default engine displacement [l]
unsigned int motorRPM = 0;                                      // Default motor speed [RPM]


int inj_status=WAITING;

// Initialization
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);                      // LCD setup

// Functions

void display_rotate(void)
{
  if (display_message < 2)
    {display_message++;}
  else
    {display_message = 0;}
  Serial.println (display_message);
}

void display_print()
{
  lcd.clear();
  switch(display_message){
    case 0:
      lcd.setCursor(0, 0);
      lcd.print ("MAF");
      lcd.setCursor(0, 1);
      lcd.print(MAF,4);
      break;
    case 1:
      lcd.setCursor(0, 0);
      lcd.print ("Temperature");
      lcd.setCursor(0, 1);
      //lcd.print(String((temp-273))+String(" C"));
      lcd.print(String((temp))+String(" K"));
      break;
    case 2:
      lcd.setCursor(0, 0);
      lcd.print ("Pressure");
      lcd.setCursor(0, 1);
      lcd.print(P);
      break;
    default:
      break;
  }
  
}
float A_calc (float P, float V, float T)
{
  float result;
  result = ((P * V) / (R * T));
  return result;
}

void callback(void) {
  if (refresh_counter < display_refresh)
  {
    refresh_counter ++;
  }
  else
  { 
    display_print();
    refresh_counter = 0;
  }
}

void out_serial(float value)
{
  Serial.print(value);
}

void read_button(int reading)
{
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {          // Check if key was pressed longer than DebounceTime to remove flickers
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == HIGH) {
        display_rotate();
      }
    }
  }
  lastButtonState = reading;
}

float getVEfromLookupTable (unsigned int motorRPM, float P)   
{
  motorRPM=motorRPM/500+1;                                    // Matrix column calculation
  int pre_value = 0;
  for(int i = 0; i < 9; i++)
  { 
    if ( (P*14.7) >= VE_Matrix[i][0] )                        // [atm] to [PSI] conversion included (14.7PSI=1atm)
    {
      pre_value=i;
    }
  }
  return VE_Matrix[pre_value][motorRPM]/100;
}

// Setup Routine
void setup() 
{
  pinMode(buttonPin, INPUT);                                   // Initialize the pushbutton pin as an input
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);                                          // Initialize Serial Port - 9600 baudrate.
  lcd.begin(16, 2);                                            // Set up the LCD's number of columns and rows
  Timer1.initialize(15);                                       // initialize timer1, 15us
  Timer1.attachInterrupt(callback);                            // attaches callback() as a timer overflow interrupt
  lcd.print ("Initialized");
  Serial.println("Initializing...");
}

// Main Routine
void loop()
{
  read_button(digitalRead(buttonPin));

  float temp_counts = analogRead(analog_temp);                  // Read TEMP counts from A0 pin
  temp = (float) (((temp_counts/1023)*TEMP_RANGE)+TEMP_MIN);     // Converts ADC counts into temperature [ºK]

  float volEfficiency = getVEfromLookupTable(motorRPM, P);      // Gets VE depending on motor speed and pressure
  V = engineDispl*volEfficiency/1000;
  
  MAF=(float)A_calc(P, V, temp);                                // Calculate Air/Fuel Mixture (n), temp is converted to C based on ADC counts
  float A = (MAF*28.966)/((RPM/60)*(N/2));                      // Air/Fuel Mixture
  //Serial.println(String("Air/Fuel Mixture: ")+String(A,10));
  float F = A / AFR;                                             // Fuel Per Cilinder
  //Serial.println(String("Fuel Per Cilinder: ")+String(F,10));
  float ti = F / Rf;                                             //Injector Duty Time
  Serial.println(ti,4);
  //Serial.println(String("Injector Duty Time: ")+String(ti,10));     
}

