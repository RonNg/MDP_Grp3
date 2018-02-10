#include <SharpIR.h>
#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>

#include "RobotMotor.h"
#include "RobotSensor.h"



RobotMotor motor;

/*==
2YK Sensors range 20-150cm (Big sensor)

21YK Sensor range 10-80cm (Small Sensor)
==*/

SharpIR ir_FL(GP2Y0A21YK0F, A0);
SharpIR ir_FR(GP2Y0A21YK0F, A1);
//------------------Motor Pins ---------------------//
const int M1A = 3;
const int M1B = 5;
const int M2A = 11;
const int M2B = 13;

//Encoder rising edge tick++
void m1Change() { motor.M1Change(); }
void m2Change() { motor.M2Change(); }




void RPMBenchtest()
{
	int setSpeed = 200;
	for (int i = 0; i < 6; ++i)
	{
		motor.GetMotor().setSpeeds(setSpeed, setSpeed);
		delay(1005);

		motor.CalcTicks();
		motor.GetMotor().setBrakes(400, 400);
		delay(1005);
	}

	motor.GetMotor().setBrakes(400, 400);
}

void CalibrationRPM()
{
	motor.ForwardCalibration(80);
}

void Calibrate_Grid(int setPoint)
{
	int calibratePower = 200;
	while (1)
	{
		double diff = abs(ir_FL.getDistance() - ir_FR.getDistance());
		
		if (diff == 0)
		{
			motor.GetMotor().setBrakes(400, 400);
			break;
		}
		
		if (ir_FL.getDistance() > ir_FR.getDistance()) //Left more front than right
		{
		}
	}
}

void setup()
{
	Serial.begin(9600);
	Serial.println("Encoder Interrupt Test:");
	enableInterrupt(M1A, m1Change, CHANGE);
	enableInterrupt(M1B, m1Change, CHANGE);

	enableInterrupt(M2A, m2Change, CHANGE);
	enableInterrupt(M2B, m2Change, CHANGE);

	motor.begin();
	//sensor.begin();

	/*for (int i = 0; i < 4; ++i)
	{
	motor.GetMotor().setSpeeds(i*70, i * 70);
	delay(1);
	}*/

	//Calibrate_Grid(10);

	motor.Forward(10);
}

int prevTime = 0;
void loop()
{
	//To update RPM for use by PID
	//motor.CalcTicks();
	//sensor.GetDistance(RobotSensor::FRONT_LEFT);

	//if ((millis() - prevTime) >= 500) // print very 500 ms
	//{
	//	Serial.print("FL: ");
	//	Serial.print(ir_FL.getDistance());

	//	Serial.print("FR: ");
	//	Serial.print(ir_FR.getDistance());

	//	Serial.println();

	//	prevTime = millis();
	//}
}

