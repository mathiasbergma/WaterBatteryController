#include <Arduino.h>
#include <PID_v1.h>
#include <Adafruit_I2CDevice.h>
#include <SPI.h>
#include <Wire.h>
#include "Timer.h" //Arduino library: Jack Christensen

#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SH1106 display(OLED_RESET);

#define LOGO16_GLCD_HEIGHT 64 
#define LOGO16_GLCD_WIDTH  128 

#define PWM_OUTPUT 6 //Output pin for PWN control
#define AMPSPIN A0  //ADC pin for current measurement
#define TEMPPIN A2  //ADC pin for temp measurement

#define TICKTEMP 1/12 //Ticks per degree
#define TANKSIZE 50

#define INTERNALREF 1083L           //Internal reference voltage
#define AVERAGE     1000            //Amount of averaging on all measurements        
#define LONGPRESS   1000            //Longpress time
#define SHORTPRESS  100             //Shortpress time


//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=20, Ki=1000, Kd=0.1;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

Timer t;

void printTopRight(float power);
void rightJustified(float num, int width, int dec);
void printButtom(float val,const char c[]);
void printTopLeft(float amps);
void printTopMid(float voltage);
void getBandgap(void); //https://gist.github.com/ronnyandre/840cb7c8f872148681ebba6a8008c530
void pressed(void);
double readI(int samples);
double readT(int samples);

byte ValResetPin = 5; //Wh reset pin
int ampsmid = 390;    //Value to withdraw from Amps ADC reading. Must be a variable  as the value is changed by long press "zeroing"
double amps = 0.0;

float battVolts;
float amplification = 1;
double PWMpercent = 0;

double temp = 0;

void setup()
{
  //initialize the variables we're linked to
  Input = amps;
  Setpoint = 1.0;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  Serial.begin(9600);
  Serial.print("Input (A),");
  Serial.print("Output (%)");
  Serial.println("Temp");

  pinMode(ValResetPin, INPUT_PULLUP);
  pinMode(AMPSPIN, INPUT);
  pinMode(TEMPPIN, INPUT);
  
  t.every(1000, getBandgap);

  display.begin(SH1106_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)

  // Clear the buffer
  display.clearDisplay();
  
  //Get system voltage based on known internal reference
  getBandgap();
}

void loop()
{
  amps = readI(AVERAGE);
  temp = readT(50);
  Input = amps;

  if (temp < 60)
  {
    myPID.Compute();
    analogWrite(PWM_OUTPUT, Output);
  }
  else
  {
    Output = 0;
    analogWrite(PWM_OUTPUT, Output);
  }
  
  PWMpercent = 100.0/256.0*Output;


  Serial.print(Input);
  Serial.print(",");
  Serial.print(PWMpercent);
  Serial.print(",");
  Serial.println(temp);

  if (!digitalRead(ValResetPin)) //If button pressed
   {
     pressed();
   }

   
  t.update();
  
  printTopLeft(amps);
  //printTopMid(battVolts);

  printTopRight(temp);
  printButtom(PWMpercent,"%");

  
}

double readI(int samples)
{
  double reading = 0;
  for (int i = 0; i < samples; i++)
  {
    reading += analogRead(AMPSPIN) - ampsmid;
  }
  
  return reading = (reading/samples)/amplification;
}
double readT(int samples)
{
  double reading = 0;
  for (int i = 0; i < samples; i++)
  {
    reading += analogRead(TEMPPIN);
  }
  
  return (reading/samples) * TICKTEMP;
}

void printTopRight(float power)
{
  display.setTextSize(1);
  display.setCursor(85,1);
  display.setTextColor(WHITE,BLACK); //Clear previous text and prevent flickering
  rightJustified(power, 5, 3); //Print spaces for a 5 position number, decimal point included
  display.print(power);
  display.print("C");
  display.display();
}
void printTopMid(float val)
{
  display.setTextSize(1);
  display.setCursor(46,1);
  display.setTextColor(WHITE,BLACK);
  rightJustified(val, 1, 2);//Print spaces for a 5 position number, decimal point included
  display.print(val);
  display.print("V");
  display.display();
  return;
}

void printTopLeft(float amps)
{
  display.setTextSize(1);
  display.setCursor(0,1);
  display.setTextColor(WHITE,BLACK); 
  rightJustified(amps, 2, 2);//Print spaces for a 4 position number, decimal point included
  display.print((amps)); 
  display.print("A");
  display.display();

  return;
}

void printButtom(float val,const char c[])
{
  display.setTextSize(2);
  display.setCursor(0,32);
  display.setTextColor(WHITE,BLACK);
  display.print("PWM: ");
  rightJustified(val, 3, 0);//Print spaces for a 7 position number, decimal point included
  display.print(val,0); 
  display.print(c);
  display.display();

  return;
}

void rightJustified(float num, int width, int dec) //Prints leading spaces
{
  int numWidth = 0;

  if (num < 10)
  {
    numWidth += width - (1+dec);
  }
  else if (num > 10 && num < 99.5)
  {
    numWidth += width - (2 + dec);
  }
  else if (num >= 99.5 && num < 1000)
  {
    numWidth += width - (3 + dec);
  }
  for (int i = 0; i < numWidth; i++)
  {
    display.print(" "); 
  }
  return;
}

void getBandgap(void) //https://gist.github.com/ronnyandre/840cb7c8f872148681ebba6a8008c530
   {
       
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // For mega boards
    const long InternalReferenceVoltage = 1115L;  // Adjust this value to your boards specific internal BG voltage x1000
       // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc reference
       // MUX4 MUX3 MUX2 MUX1 MUX0  --> 11110 1.1V (VBG)         -Selects channel 30, bandgap voltage, to measure
    ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR)| (0<<MUX5) | (1<<MUX4) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
 
#else
    // For 168/328 boards
    const long InternalReferenceVoltage = INTERNALREF;  // Adjust this value to your boards specific internal BG voltage x1000
       // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
       // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
    ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
     
#endif
    delay(50);  // Let mux settle a little to get a more stable A/D conversion
       // Start a conversion  
    ADCSRA |= _BV( ADSC );
       // Wait for it to complete
    while( ( (ADCSRA & (1<<ADSC)) != 0 ) );
       // Scale the value
    for (int i = 0; i < AVERAGE; i++)
    {
      battVolts += (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L; // calculates for straight line value
    }
    battVolts /= AVERAGE;
    battVolts /=100;
    amplification = (1024 / (battVolts*1000)) * 69;
    return;
   }
   
void pressed(void)
{
  long timeStart = millis(); //Start timestamp

  while (!digitalRead(ValResetPin)) //Loop while pressed
  {
    continue;
  }

  long timeStop = millis(); //Stop timestamp

  if (timeStop > timeStart + LONGPRESS) //If LONGPRESS time has passed
  {
    ampsmid = ampsmid + amps * amplification; //Calculate new zero point
  }
}