#include <Arduino.h>
#include <PID_v1.h>
#include <Adafruit_I2CDevice.h>
#include <SPI.h>
#include <Wire.h>
#include "Timer.h" //Arduino library: Jack Christensen

#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h> //Alternative library for different screen

#include <Adafruit_SH1106.h> //https://github.com/wonho-maker/Adafruit_SH1106.git

#define PIDSP -0.1


#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_HEIGHT 64
#define SCREEN_WIDTH  128 

//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SH1106 display(OLED_RESET);


#define PWM_OUTPUT  6     //Output pin for PWN control
#define AMPSPIN     A0    //ADC pin for current measurement
#define TEMPPIN     A2    //ADC pin for temp measurement

#define TICKTEMP    1/12  //Ticks per degree
#define TANKSIZE    50.0  //Used for energy calculation

#define AVERAGE     1000  //Amount of averaging on all measurements        


#define LOWTEMP     40.0 //Lower temperature limit
#define SHEAT       4182.0 //Specific heat of water in Joule/degree/kg
#define TOWATTHR    0.00027777777777778


//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=1, Ki=100, Kd=0.1;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);



void printTopRight(float val, const char c[]);
void rightJustified(float num, int width, bool neg);
void printButtom(float val,const char c[]);
void printTopLeft(float val, const char c[]);
void printTopMid(float val, const char c[]);
double readI(int samples);
double readT(int samples);


int ampsmid = 512;    //Value to withdraw from Amps ADC reading. Must be a variable  as the value is changed by long press "zeroing"
double amps = 0.0;
int analogAmp;

double energy = 0;

double amplification = (1024 / 5000.0) * 69;
double PWMpercent = 0;

double temp = 0;

void setup()
{
  //initialize the variables we're linked to
  Input = amps;
  Setpoint = PIDSP;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  Serial.begin(9600);
  Serial.print("Input (A),");
  Serial.print("Output (%)");
  Serial.println("Temp");

  pinMode(AMPSPIN, INPUT);
  pinMode(TEMPPIN, INPUT);

  //display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)

  display.begin(SH1106_SWITCHCAPVCC, 0x3C);

  // Clear the buffer
  display.clearDisplay();
}

void loop()
{
  amps = readI(AVERAGE);
  temp = readT(100);
  Input = amps;
  //Serial.println(ampsmid);
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
  
  //Calculate energy in tank
  if (temp > 40)
  {
    energy = SHEAT * TANKSIZE * (temp - LOWTEMP) * TOWATTHR;
  }
  else
  {
    energy = 0;
  }
  


  PWMpercent = 100.0/256.0*Output;

  //Send relevant data on Serial
  Serial.print(Input);
  Serial.print(",");
  Serial.print(PWMpercent);
  Serial.print(",");
  Serial.println(temp);
  
  //Show information on screen
  printTopLeft(amps, "A");
  printTopMid(energy, "Wh");
  printTopRight(temp, "c");
  printButtom(PWMpercent,"%");

  
}

double readI(int samples)
{
  double reading = 0;
  for (int i = 0; i < samples; i++)
  {
    reading += analogRead(AMPSPIN) - ampsmid;
  }
  analogAmp = analogRead(AMPSPIN);
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

void printTopRight(float val, const char c[])
{
  display.setTextSize(1);
  display.setCursor(100,3);
  display.setTextColor(WHITE,BLACK); //Clear previous text and prevent flickering
  rightJustified(val, 2, 0); //Print spaces for a 5 position number, decimal point included
  display.print(val,0);
  display.print(c);
  display.display();
  return;
}
void printTopMid(float val, const char c[])
{
  display.setTextSize(1);
  display.setCursor(46,3);
  display.setTextColor(WHITE,BLACK);
  rightJustified(val, 1, 0);//Print spaces for a 5 position number, decimal point included
  display.print(val);
  display.print(c);
  display.display();
  return;
}

void printTopLeft(float val, const char c[])
{
  display.setTextSize(1);
  display.setCursor(0,3);
  display.setTextColor(WHITE,BLACK); 
  rightJustified(val, 2, 1);//Print spaces for a 4 position number, decimal point included
  display.print(val,1); 
  display.print(c);
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

void rightJustified(float num, int width, bool neg) //Prints leading spaces
{
  int numWidth = 0;

  if (neg)
  {
    numWidth = 1;
  }

  if (abs(num) < 9.5)
  {
    numWidth += width - 1;
  }
  else if (abs(num) > 9.5 && abs(num) < 99.5)
  {
    numWidth += width - 2;
  }
  else if (abs(num) >= 99.5 && abs(num) < 1000)
  {
    numWidth += width - 3;
  }
  if (num >= 0)
  {
    for (int i = 0; i < numWidth; i++)
    {
      display.print(" "); 
    }
  }
  else
  {
    for (int i = 1; i < numWidth; i++)
    {
      display.print(" "); 
    }
  }
  
  return;
}