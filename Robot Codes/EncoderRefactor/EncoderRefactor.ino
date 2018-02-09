#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>
#include "RobotMotor.h"



RobotMotor motor;

//------------------Motor Pins ---------------------//
const int M1A = 3;
const int M1B = 5;
const int M2A = 11;
const int M2B = 13;

//Encoder rising edge tick++
void m1Change() { motor.M1Change(); }
void m2Change() { motor.M2Change(); }


void setup()
{
	Serial.begin(9600);
	Serial.println("Encoder Interrupt Test:");
	enableInterrupt(M1A, m1Change, CHANGE);
	enableInterrupt(M1B, m1Change, CHANGE);

	enableInterrupt(M2A, m2Change, CHANGE);
	enableInterrupt(M2B, m2Change, CHANGE);

	motor.begin();
}

void RPMBenchtest()
{
	int setSpeed = 250;
	for (int i = 0; i < 5; ++i)
	{
		motor.GetMotor().setSpeeds(setSpeed, setSpeed);
		delay(1005);

		motor.CalcTicks();
		motor.GetMotor().setBrakes(400, 400);	
	}
}

bool runOnce = false;

void loop()
{
	//To update RPM for use by PID
	motor.CalcTicks();

	if (!runOnce)
	{
		RPMBenchtest();
		runOnce = false;
	}


}
