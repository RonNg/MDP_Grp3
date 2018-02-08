#include <EnableInterrupt.h>
#include <Encoder.h>
#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield md;

//------------------Motor Pins ---------------------//
const int M1A = 3;
const int M1B = 5;
const int M2A = 11;
const int M2B = 13;

double m1Ticks = 0;
double m2Ticks = 0;

double m1RPM = 0; 
double m2RPM = 0;

int m1TargetCounter = 0; //For counting to target ticks bnased on distance
int m2TargetCounter = 0;


#define TICK_REFRESH_INTERVAL 1000 //ms
#define TICKS_PER_CM 119 
#define TICKS_PER_REV 2248.86

unsigned long lastTime = 0;


double m1Speed = 0;
double m2Speed = 0;

double errSum = 0;
double lastErr = 0;

//Encoder rising edge tick++
void m1Change(){m1Ticks++; m1TargetCounter++;}
void m2Change(){m2Ticks++; m2TargetCounter++;}

void calcTicks(int refreshInterval)
{
    if ((millis() - lastTime) >=  refreshInterval)
    {
      /*
      Serial.print("Left Tick: ");
      Serial.print(m1Ticks);
      
      Serial.print(", Right Ticks: ");
      Serial.print(m2Ticks);
      Serial.println();
    */
      
      m1RPM = ((m1Ticks*(60000/refreshInterval))/TICKS_PER_REV); //Extrapolate RPM
      m2RPM = ((m2Ticks*(60000/refreshInterval))/TICKS_PER_REV); //100ms * 600ms = 1 minute (interpolate RPM)
      m1Ticks = 0;
      m2Ticks = 0;
      
      lastTime = millis();
    }   
}

void ResetPID()
{
  errSum = 0;
  lastErr = 0;
}

//Direction: True is forward, false is backward
double computePID(double consKp, double consKi, double consKd, char direction, double targetRPM, int motor)
{
  int currentRPM;
  
  if(motor == 1)
    currentRPM = m1RPM;
  else if(motor == 2)
    currentRPM = m2RPM;
    
  //Setpoint - Input(Target Tick)
  double error = targetRPM - currentRPM;

  //Integral
  errSum += error; 
  
  //Derivative
  double dErr = error - lastErr;
  lastErr = error;

  return ((consKp * error) + (consKi * errSum) + (consKd * dErr));
}


void TestStraight()
{
    calcTicks (TICK_REFRESH_INTERVAL);

    /*
    Serial.print("Left RPM: ");
    Serial.print(m1RPM);
    
    Serial.print(", Right RPM: ");
    Serial.print(m2RPM);
    Serial.println();
    */

    md.setSpeeds(200, 200);
    //m1Speed += computePID(0.00005, 0, 0.08, 'f', 80, 1);
    //m2Speed += computePID(0.00008, 0, 0.08, 'f', 80, 2);

    if(m1Speed <= 0)
      m1Speed = 0;

    if(m2Speed <=0)
      m2Speed = 0;
      
    //md.setSpeeds(m1Speed, m2Speed);     
}

void Forward(double cm)
{
  int targetTick = cm * TICKS_PER_CM;
  m1TargetCounter = m2TargetCounter = 0;

  Serial.println("Start of forward");

  //Open Loop

  if(m1Speed == 0 || m2Speed == 0)
  {
    for(int i = 0; i < 4; ++ i)
    {
      md.setSpeeds(i*50, i*50);
      delay(50);
    }
  }

  m1Speed = 200;
  m2Speed = 200;
  
  while(m1TargetCounter < targetTick || m2TargetCounter < targetTick)
  {
    calcTicks (TICK_REFRESH_INTERVAL);
  
    Serial.print("Left RPM: ");
    Serial.print(m1RPM);
    
    Serial.print(", Right RPM: ");
    Serial.print(m2RPM);
    Serial.println();

    m1Speed += computePID(0.0025, 0, 0.0005, 'f', 40, 1);
    m2Speed += computePID(0.008, 0, 0.0005, 'f', 40, 2);
    
    if(m1Speed <= 0)
      m1Speed = 0;

    if(m2Speed <=0)
      m2Speed = 0;
      
    md.setSpeeds(m1Speed, m2Speed);     
  }

  Serial.print(m1Speed);
  Serial.print(", ");
  Serial.print(m2Speed);
  
  md.setBrakes(390, 400);
}

void TurnRight()
{
   //Diameter of wheel to wheel is 17cm
  //For a 90 deg turn, both wheels must turn the robot 45 degs in each direction
  //By using the formula Pi * 17 * 45/360 
  //We can find that the distance each wheel must travel is 6.67cm

  //Turn 90 deg
  //int targetTick = 6.67 * TICKS_PER_CM;
  int targetTick = 13 * TICKS_PER_CM;
  m1TargetCounter = m2TargetCounter = 0;

 
  if(m1Speed == 0 || m2Speed == 0)
  {
    for(int i = 0; i < 3; ++ i)
    {
      md.setSpeeds(i*50, -i*50);
      delay(50);
    }
    m1Speed = 200;
    m2Speed = 200;
  }
  
  while(m1TargetCounter < targetTick - 90 || m2TargetCounter < targetTick)
  {
    calcTicks (TICK_REFRESH_INTERVAL);
  
    Serial.print("Left RPM: ");
    Serial.print(m1RPM);
    
    Serial.print(", Right RPM: ");
    Serial.print(m2RPM);
    Serial.println();

    m1Speed += computePID(0.0020, 0, 0.0005, 'f', 40, 1);
    m2Speed += computePID(0.0025, 0, 0.0005, 'f', 40, 2);
    
    if(m1Speed <= 0)
      m1Speed = 0;

    //Right turn the right motor must go backwards
    if(m2Speed <= 0)
      m2Speed = 0;
 
    md.setSpeeds(m1Speed, -m2Speed);     
  }

  Serial.print(m1Speed);
  Serial.print(", ");
  Serial.print(m2Speed);
  
  md.setBrakes(400, 370);
}

void TurnLeft()
{
  
}
void Turn (int angle)
{
 TurnRight();
}



void setup() 
{
  Serial.begin(9600);
  Serial.println("Encoder Interrupt Test:");
  enableInterrupt(M1A, m1Change, CHANGE);
  enableInterrupt(M1B, m1Change, CHANGE);
  
  enableInterrupt(M2A, m2Change, CHANGE);
  enableInterrupt(M2B, m2Change, CHANGE);
  md.init();
}

int i = 0;
char input;
char commands[7] = {'f', 'r', 'f', 'r', 'f', 'r', 'f'};

void loop() 
{  
  //To update RPM for use by PID
  calcTicks (TICK_REFRESH_INTERVAL);

  int commandSize = 7;
  //TestStraight();
  input = commands[i];

  ++ i;




  if(i > 7)
  {
    while(1);
  }
    
  switch(input)
  {
    case 'f':
    Forward(10);
    delay(1000);
    ResetPID();
    m1Speed = m2Speed = 0;    
    break;

    case 'r':
    Turn(45);
    delay(1000);
    ResetPID();
    m1Speed = m2Speed = 0;
    break;
  }
}
