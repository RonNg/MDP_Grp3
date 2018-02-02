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


double masterSpeed = 300;
double slaveSpeed = 300;

unsigned long lastTime;
double errSum = 0;
double lastErr = 0;



void encoder1Change()
{
  m1Ticks++;
}

void encoder2Change()
{
  m2Ticks++;

}

void calculateTicksPerSec()
{
    Serial.print("Left = ");
    Serial.print(m1Ticks);

    Serial.print(", Right = ");  
    Serial.print(m2Ticks);
    Serial.println();

    m1LastTick = m1Ticks;
    m2LastTick = m2Ticks;
  
    m1Ticks = 0;
    m2Ticks = 0;
}


//Direction: True is forward, false is backward
void computePID(double consKp,double consKi, double consKd, char  direction)
{
  //Since Right ist he master and left is the slave, we calculate error according to right side
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);

  //Setpoint - Input(Target)
  
  double error = (m1LastTick - m2LastTick);
  
  errSum += error; //Integral
  double dErr = error-lastErr;

  double gap = abs(m1LastTick - m2LastTick);
  double output = 0;
  
  output = consKp * error + consKi * errSum + consKd * dErr; 

/*
  if(direction)
  {     
    slaveSpeed -= output; //Since slave is faster, we need to slow down. Kp must be proportionate to the error. If error is expected to be huge (for high speed
    
  }
  else
  {
      
    slaveSpeed += output;
    
       if(slaveSpeed>0)
      slaveSpeed = 0;
  }
*/

  switch(direction)
  {
    case 'f':
    slaveSpeed -= output;
    break;
    
    case 'r':
    slaveSpeed += output;
    break;

    default:
    break;

    case'r':
    
  }
  
  
  lastErr = error;
  lastTime = now;
}

void setup() 
{
  Serial.begin(9600);
  Serial.println("Encoder Interrupt Test:");
  enableInterrupt(M1A, encoder1Change, RISING);
  enableInterrupt(M2A, encoder2Change, RISING);

  //For reverse
  masterSpeed = -300;
  slaveSpeed = -300;
  
  md.init();
  md.setSpeeds(slaveSpeed,masterSpeed);
}


void loop() 
{  
  calculateTicksPerSec();

  computePID(4, 0.5, 3, 'r');
    
  md.setM1Speed(slaveSpeed);

  delay(100); //calculate per 0.1 second
}
