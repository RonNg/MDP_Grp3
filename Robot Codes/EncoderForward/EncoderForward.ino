#include <EnableInterrupt.h>
#include <Encoder.h>
#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;

//------------------Motor Pins ---------------------//
const int M1A = 3;
const int M1B = 5;
const int M2A = 11;
const int M2B = 13;

int m1Ticks = 0;
int m2Ticks = 0;

int m1LastTick = 0; //Ticks since last refresh interval
int m2LastTick = 0;

int m1CalTick = 0; //For tick calibration
int m2CalTick = 0; 

int m1TicksElapsed = 0; //For setting distance movement

int counter = 0; //Temp

#define TICK_REFRESH_INTERVAL 100 //ms
#define TICK_PER_CM 22 // 415 / 18.84
unsigned long lastTime = 0;


double masterSpeed = 300;
double slaveSpeed = 300;

double errSum = 0;
double lastErr = 0;

//Encoder rising edge tick++
void encoder1Change(){m1Ticks++; m1CalTick++; m1TicksElapsed++;}
void encoder2Change(){m2Ticks++; m2CalTick++;}

void calcTicks(int refreshInterval)
{
    if ((millis() - lastTime) >=  refreshInterval)
    {
      m1LastTick = m1Ticks;
      m2LastTick = m2Ticks;
      m1Ticks = 0;
      m2Ticks = 0;

      lastTime = millis();
    }   
}

void ticksForCalibration()
{
  
}

void ResetPID()
{
  errSum = 0;
  lastErr = 0;
}

//Direction: True is forward, false is backward
double computePID(double consKp, double consKi, double consKd, char direction)
{
  //Setpoint - Input(Target Tick)
  double error = (m1LastTick - m2LastTick);

  //Integral
  errSum += error; 
  
  //Derivative
  double dErr = error - lastErr;
  lastErr = error;
    
  return ((consKp * error) + (consKi * errSum) + (consKd * dErr));
}


void Forward(double cm)
{
  m1TicksElapsed = 0;

  if(counter > 3)
  {
    ResetPID();
    counter = 0;
  }  
   
  int targetTick = cm * TICK_PER_CM;
  
  //Serial.println("Start of forward");
  

  while(m1TicksElapsed < targetTick)
  {
    calcTicks (TICK_REFRESH_INTERVAL);
  
    Serial.print("Left: ");
    Serial.print(m1LastTick);
    
    Serial.print(", Right: ");
    Serial.print(m2LastTick);
    Serial.println();
  
    //slaveSpeed +=  computePID(0.25, 0.0025, 0.5, 'f'); 

    //Test for 15cm
    slaveSpeed +=  (computePID(0.25, 0.0008, 0.7, 'f'));   
    //Prevents reversing of wheel
    if(slaveSpeed <= 0)
        slaveSpeed = 0;
      
    md.setSpeeds(masterSpeed, slaveSpeed*1.13);     
  }

  Serial.println(m1TicksElapsed);
 

  md.setSpeeds(0, 0);
  counter++;
  
}

void CalibrateOneRotation()
{
  //One rotation is 18.5cm since wheel diameter is 6cm
  md.setM1Speed(300);

  delay(500);

  Serial.print("Number of ticks: ");
  Serial.println(m1CalTick);

  md.setM1Speed(0);
  
}

void setup() 
{
  Serial.begin(9600);
  Serial.println("Encoder Interrupt Test:");
  enableInterrupt(M1A, encoder1Change, RISING);
  enableInterrupt(M2A, encoder2Change, RISING);
  md.init();

  //CalibrateOneRotation();
}

void loop() 
{  
 
  char input = 'f';
  switch(input)
  {
    case 'f':
    Forward(15);
    delay(3000);
    break;

    case 'r':
       break;
  }
}
