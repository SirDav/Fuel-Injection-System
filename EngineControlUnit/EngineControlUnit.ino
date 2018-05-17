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
const int buttonPin = 2;                                        // External button for menu
const int ledPin =  13;                                         // the number of the LED pin

// Global Variables
int lastButtonState = LOW;
int buttonState;
int display_refresh = 1000;                                      // Display Refresh Rate [ms]
int refresh_counter = 0;
unsigned long lastDebounceTime = 0;                             // Last Time the Key was Toggled
unsigned long debounceDelay = 50;                               // Debounce Time; Increase if the Input Flickers
int display_message = 0;                                        // Display Screen in use (Value to Print)
int temp = 273;                                                 // Default Temperature: 273K = 0C
int N = 4;                                                      // Number of Cilinders
int P = 1;                                                      // Default Atmospheric Pressure
float R = 0.18;                                                      // Default Gasoline Molecular Weight
int V = 1;                                                      // Default Air Mass Volume
float MAF = 0;
int Inj_output = 3;                                             // Output pin for Injector

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
      lcd.print(String((temp-273))+String(" C"));
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
float A_calc (int P, int V, int T)
{
  float result;
  result = ((P * V) / (R * T));
  return result;
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = 50000U;
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

// Setup Routine
void setup() 
{
  pinMode(buttonPin, INPUT);                                   // Initialize the pushbutton pin as an input
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);                                          // Initialize Serial Port - 9600 baudrate.
  lcd.begin(16, 2);                                            // Set up the LCD's number of columns and rows

  TCCR1A = 0;           // you have to change the timer mode before changes
  TCCR1B = 0;           // stop the timer
  TIFR1 |= _BV(TOV1);   // clear the overflow interrupt flag by writing 1 to it
  TCNT1  = 50000U;      // tick in around 1ms / this makes more sense if it's set correctly
  TCCR1B = _BV(CS10);   // no prescaler ?????  REALLY???? this one is prescale by 8
  TIMSK1 |= _BV(TOIE1); // enable timer overflow interrupt
    
  lcd.print ("Initialized");
  Serial.println("Initializing...");
}

// Main Routine
void loop()
{
  temp = analogRead(analog_temp);                              // Read TEMP from A0 pin
  MAF=(float)A_calc(P, V, temp*5);                             // Calculate Air/Fuel Mixture (n), temp is converted to C based on ADC counts
  //analogWrite(Inj_output, temp/4);
  int reading = digitalRead(buttonPin);
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

