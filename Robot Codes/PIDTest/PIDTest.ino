#include <PID_v1.h>
#include <DualVNH5019MotorShield.h>
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


PID myPID(&m1Ticks, &m1LastTick, &m2LastTick, 0.5, 0, 0, DIRECT);

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}

void calculateTicks()
{
    Serial.print("Left = ");
    Serial.print(m1Ticks);
    Serial.println();

    Serial.print(", Right = ");  
    Serial.print(m2Ticks);
    Serial.println();

    m1LastTick = m1Ticks;
    m2LastTick = m2Ticks;
  
    m1Ticks = 0;
    m2Ticks = 0;
}


void setup() 
{
  Serial.begin(9600);
  md.init();

  
  md.setSpeeds(slaveSpeed, masterSpeed);

  
}


void loop() 
{
  calculateTicks();
  delay(100);

  myPID.Compute();

  
  
  md.setM1Speed(
  
}
