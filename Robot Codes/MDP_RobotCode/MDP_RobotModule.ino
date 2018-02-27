#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>
#include "RobotMotor.h"
#include "RobotSensor.h"
#include <math.h>
#include <SharpIR.h>

RobotMotor motor;
#define DIST_BETWEEN_FRONT_SENSOR 10 //distance (lens to lens) of IR_FL and IR_FR

/*==
2YK Sensors range 20-150cm (Big sensor)

21YK Sensor range 10-80cm (Small Sensor)
==*/

//GP2Y0A21YK0F = short 10cm to 80cm
//GP2Y0A2YK0F = long 20cm to 150cm
SharpIR ir_FL(GP2Y0A21YK0F, A0);
SharpIR ir_FR(GP2Y0A21YK0F, A1);
SharpIR ir_FC(GP2Y0A21YK0F, A2); //Front center
SharpIR ir_RF(GP2Y0A21YK0F, A3); //Right Forward
SharpIR ir_RB(GP2Y0A21YK0F, A4); //Right Back




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
void Calibrate_Angle()
{
	double leftSensor;
	double rightSensor;
	for (int i = 0; i < 10; ++ i )
	{
		//Align both sensors forward first, regardless whether its far or near
		leftSensor = ir_FL.getDistance(); 
		rightSensor = ir_FR.getDistance();

		double sensorDiff = abs(leftSensor - rightSensor);

		if (sensorDiff <= 0.0)
			break;
	
		//Atan returns radian. Convert rad to deg by multiplying 180/PI
		double angle = atan(sensorDiff / 14) * 180 / PI;
		

		//Left is further
		if (leftSensor > rightSensor)
		{
			//Turn right
			//Serial.print("   Turn Right");
			motor.Turn(angle);

		}
		else //Right is further
		{
			//Turn left
			//Serial.print("   Turn Left");
			motor.Turn(-angle);
		}
	}

	Serial.println("End calibration");
}

//Calibrates right side to align straight
void Calibrate_SideAngle()
{
	double leftSensor;
	double rightSensor;
	for (int i = 0; i < 10; ++i)
	{
		//Align both sensors forward first, regardless whether its far or near
		leftSensor = ir_RF.getDistance();
		rightSensor = ir_RB.getDistance();

		double sensorDiff = abs(leftSensor - rightSensor);

		if (sensorDiff <= 0.0)
			break;

		//Atan returns radian. Convert rad to deg by multiplying 180/PI
		double angle = atan(sensorDiff / 14) * 180 / PI;


		//Left is further
		if (leftSensor > rightSensor)
		{
			//Turn right
			//Serial.print("   Turn Right");
			motor.Turn(angle);

		}
		else //Right is further
		{
			//Turn left
			//Serial.print("   Turn Left");
			motor.Turn(-angle);
		}
	}

	Serial.println("End calibration");
}

//Move forward until one sensor reads setPoint distance
void Calibrate_Forward(int setPoint) 
{

	for (int i = 0; i < 5; ++i)
	{
		double frontSensor = ir_FC.getDistance();

		//Distance between robot and setpoint
		//If negative, it means that the robot is too near from the setpoint
		//If positive, it means that the robot is too far from the setpoint
		double distance = frontSensor - setPoint;

		if (distance == 0)
			break;

		Serial.print("Distance: ");
		Serial.println(distance);
		if (distance > 0) //Robot away from setpoint
		{
			//Absolute distance must be passed in as a negative value will mess up the Forward function
			//to indicate reverse instead, we pass true in the second argument
			motor.Forward(abs(distance), false);
		}
		else if (distance < 0) //Robot too 
		{
			motor.Forward(abs(distance), true);
		}
	}
}



void Calibrate_Full(int setPoint)
{
	//Calibrate_Angle(setPoint);

	//while (!Calibrate_Forward(setPoint))
	//{
	//	Calibrate_Angle(setPoint);
	//}

}

void CalibrationTest()
{
	Calibrate_Angle();
	delay(100);

	Calibrate_Forward(15);
	delay(100);

	//Turn left
	motor.TurnLeft90();
	delay(100);


	//Calibrate
	Calibrate_Angle();
	delay(100);

	Calibrate_Forward(15);
	delay(100);


	//Turn left
	motor.TurnRight90();
	delay(100);


	//Calibrate
	Calibrate_Angle();
	delay(100);

	Calibrate_Forward(15);
	delay(100);

	//Turn left
	motor.TurnLeft90();
	delay(100);


	//Calibrate
	Calibrate_Angle();
	delay(100);

	Calibrate_Forward(15);
	delay(100);

	motor.TurnLeft90();
	delay(100);

	Calibrate_SideAngle();
}


void Checklist_Obstacle(int distance)
{
	int travelled = ((distance-30)/10);

	Serial.println(travelled);
	CalibrationTest();

	while (ir_FC.getDistance() > 10)
	{
		motor.Forward10(false);
		--travelled;
	}

	//Obstacle Avoidance Start
	Calibrate_SideAngle();

	motor.TurnLeft90();

	motor.Forward10(false);

	motor.Forward10(false);

	motor.TurnRight90();
	//Calibrate_SideAngle();


	for (int i = 0; i < 4; ++i)
	{
		motor.Forward10(false);
	}

	motor.TurnRight90();
	

	motor.Forward10(false);

	motor.Forward10(false);

	Calibrate_Angle();

	Calibrate_Forward(15);

	motor.TurnLeft90();

	Calibrate_SideAngle();

	//Obstacle Avoidance End
	while (travelled > 0)
	{
		motor.Forward10(false);
		--travelled;
	}
}

void setup()
{
	Serial.begin(9600);

	enableInterrupt(M1A, m1Change, CHANGE);
	enableInterrupt(M1B, m1Change, CHANGE);

	enableInterrupt(M2A, m2Change, CHANGE);
	enableInterrupt(M2B, m2Change, CHANGE);

	motor.begin();

	//Checklist_Obstacle(90);
	for (int i = 0; i < 5; ++i)
		motor.Forward10();
}

void loop()
{
	//motor.CalcTicks
	Serial.println(ir_FC.getDistance());

	delay(500);
	

}
