#include <SharpIR.h>
#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>

#include "RobotMotor.h"
#include "RobotSensor.h"

#include <math.h>


RobotMotor motor;
#define DIST_BETWEEN_FRONT_SENSOR 10 //distance (lens to lens) of IR_FL and IR_FR

/*==
2YK Sensors range 20-150cm (Big sensor)

21YK Sensor range 10-80cm (Small Sensor)
==*/

SharpIR ir_FL(A0, 1080);
SharpIR ir_FR(A1, 1080);
SharpIR ir_FC(A2, 1080); //Front center



//------------------Motor Pins ---------------------//
const int M1A = 3;
const int M1B = 5;
const int M2A = 11;
const int M2B = 13;

//Encoder rising edge tick++
void m1Change() 
{ 
	motor.M1Change(); 
}

void m2Change() 
{ 
	motor.M2Change(); 
}




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
	motor.CalibrationForward(80);
}

//Makes both sensors aligned with each other i.e. have both sensors measure same distance from the front
void Calibrate_Angle(int setPoint)
{
	double leftSensor;
	double rightSensor;
	for (int i = 0; i < 10; i++)
	{
		
		//Align both sensors forward first, regardless whether its far or near
		leftSensor = ir_FL.distance(); 
		rightSensor = ir_FR.distance();

		double sensorDiff = abs(leftSensor - rightSensor);

		if (sensorDiff <= 0.0)
			break;

		//if (sensorDiff == 0) //Aligned
		//	break;

		//Atan returns radian. Convert rad to deg by multiplying 180/PI
		double angle = atan(sensorDiff / 15) * 180 / PI;
		
		Serial.print(leftSensor);
		Serial.print(", ");
		Serial.print(rightSensor);
		Serial.print(" Sensor Diff: ");
		Serial.print(sensorDiff);


		Serial.print("  Angle: ");
		Serial.println(angle);

		//Left is further
		if (leftSensor > rightSensor)
		{
			//Turn right
			Serial.print("   Turn Right");
			motor.Turn(angle);
			delay(100);

		}
		else //Right is further
		{
			//Turn left
			Serial.print("   Turn Left");
			motor.Turn(-angle);
			delay(100);
		}
	}
}
//Move forward until one sensor reads setPoint distance
bool Calibrate_Forward(int setPoint) 
{

	double leftSensor = ir_FL.distance();

	//Distance between robot and setpoint
	//If negative, it means that the robot is too near from the setpoint
	//If positive, it means that the robot is too far from the setpoint
	double distance = leftSensor - setPoint;

	Serial.print("Distance: ");
	Serial.println(distance);
	if (distance > 0) //Robot away from setpoint
	{
		//Absolute distance must be passed in as a negative value will mess up the Forward function
		//to indicate reverse instead, we pass true in the second argument
		motor.Forward(abs(distance), false);
		return 0;
	}
	else if (distance < 0) //Robot too 
	{
		motor.Forward(abs(distance), true);
		return 0;
	}
	else
		return 1;

}

void Calibrate_Full(int setPoint)
{
	Calibrate_Angle(setPoint);

	while (!Calibrate_Forward(setPoint))
	{

		Calibrate_Angle(setPoint);

	}

}

void CalibrationTest(int setPoint)
{
	Calibrate_Angle(setPoint);

	//Turn left
	motor.Turn90(false);
	delay(100);

	//Calibrate
	Calibrate_Full(setPoint);

	//Turn right
	motor.Turn90(true);
	delay(100);

	//Calibrate
	Calibrate_Full(setPoint);

	//Turn left
	motor.Turn90(false);
	delay(100);

	//Calibrate
	Calibrate_Full(setPoint);
}

void TestSensor()
{
	Serial.println(ir_FC.distance());
}

void setup()
{
	Serial.begin(9600);

	enableInterrupt(M1A, m1Change, CHANGE);
	enableInterrupt(M1B, m1Change, CHANGE);

	enableInterrupt(M2A, m2Change, CHANGE);
	enableInterrupt(M2B, m2Change, CHANGE);

	motor.begin();




	/*for (int i = 0; i < 15; ++i)
	{
		motor.Forward10(false);
		delay(500);
	}*/

	//motor.Forward(150, false);

}

void loop()
{
	motor.CalcTicks();
	
	motor.Turn90(true);
	delay(1000);
}

