#include <Arduino.h>
#include <PID_v1.h>
#include <Adafruit_I2CDevice.h>
#include <SPI.h>
#include <Wire.h>
#include "Timer.h" //Arduino library: Jack Christensen

#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h> //Alternative library for different screen

#include <Adafruit_SH1106.h> //https://github.com/wonho-maker/Adafruit_SH1106.git


Timer t;

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

#define AVERAGE     500  //Amount of averaging on all measurements        


#define LOWTEMP     40.0 //Lower temperature limit
#define SHEAT       4182.0 //Specific heat of water in Joule/degree/kg
#define TOWATTHR    0.00027777777777778
#define AMPSMID 511


//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=7, Ki=12, Kd=0.05;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void communicate();
void printTopRight(float val, const char c[]);
void rightJustified(float num, int width, bool neg);
void printButtom(float val,const char c[]);
void printTopLeft(float val, const char c[]);
void printTopMid(float val, const char c[]);
void readI(double *ptr, const int SAMPLES);
void readTemp(double *ptr, const int SAMPLES);



double amplification = (1024 / 5000.0) * 100;
double PWMpercent = 0;
double temp = 0;
bool PIDon = false;

void setup()
{
  //initialize the variables we're linked to
  Input = 0.0;
  Setpoint = PIDSP;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  Serial.begin(9600);
 // Serial.println("Input (A), Output (%), Temp (c)");

  pinMode(AMPSPIN, INPUT);
  pinMode(TEMPPIN, INPUT);

  t.every(3000, communicate); // 2 seconds

  //display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  

  display.begin(SH1106_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x64)

  // Clear the buffer
  display.clearDisplay();

  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 6250;            // compare match register 16MHz/256/10Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt

  interrupts();             // enable all interrupts

}


ISR(TIMER1_COMPA_vect)      // timer compare B interrupt service routine
{
  readI(&Input, AVERAGE);
  readTemp(&temp, 200);
}

void loop()
{
  double energy = 0;

  if (temp > 60)
  {
    PIDon = false;
  }
  if (temp < 58)
  {
    PIDon = true;
  }

  if (PIDon)
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

  
  t.update();
  
  //Show information on screen
  printTopLeft(Input, "A");
  printTopMid(energy, "Wh");
  printTopRight(temp, "c");
  printButtom(PWMpercent,"%");

  
}
void communicate()
{
  //Send relevant data on Serial
  Serial.print("D,");
  Serial.print(Input);
  Serial.print(",");
  Serial.print(PWMpercent);
  Serial.print(",");
  Serial.println(temp);
}

void readI(double *ptr, const int SAMPLES)
{
  double reading = 0;
  for (int i = 0; i < SAMPLES; i++) //Perform reading
  {
    reading += analogRead(AMPSPIN) - AMPSMID;
  }
  *ptr = (reading/SAMPLES)/amplification; //Calculate average and assign
  return;
}

void readTemp(double *ptr, int SAMPLES)
{
  double reading = 0;
  for (int i = 0; i < SAMPLES; i++) //Perform reading
  {
    reading += analogRead(TEMPPIN);
  }
  *ptr = (reading/SAMPLES) * TICKTEMP; //Calculate average and assign
  return;
}

void printTopRight(float val, const char c[])
{
  display.setTextSize(1);
  display.setCursor(100,3);
  display.setTextColor(WHITE,BLACK); //Clear previous text and prevent flickering
  rightJustified(val, 2, 0); //Print spaces for a 2 position number, decimal point included
  display.print(val,0);
  display.print(c);
  display.display();
  return;
}
void printTopMid(float val, const char c[])
{
  display.setTextSize(1);
  display.setCursor(47,3);
  display.setTextColor(WHITE,BLACK);
  rightJustified(val, 4, 0);//Print spaces for a 5 position number, decimal point included
  display.print(val,0);
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