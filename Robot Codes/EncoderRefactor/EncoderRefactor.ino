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

//Makes both sensors aligned with each other i.e. have both sensors measure same distance from the front
void Calibrate_SensorAngle(int setPoint)
{
	double leftSensor;
	double rightSensor;
	for (int i = 0; i < 6; i++)
	{
		
		//Align both sensors forward first, regardless whether its far or near
		leftSensor = ir_FL.getDistance(); 
		rightSensor = ir_FR.getDistance();

		double sensorDiff = abs(leftSensor - rightSensor);

		if (sensorDiff <= 0.2)
			break;

		//if (sensorDiff == 0) //Aligned
		//	break;

		//Atan returns radian. Convert rad to deg by multiplying 180/PI
		double angle = atan(sensorDiff / 15.0) * 180 / PI;
		
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

		}
		else //Right is further
		{
			//Turn left
			Serial.print("   Turn Left");
			motor.Turn(-angle);
		}
	}


	//Assume that both sensors are aligned
	//Move forward to make one sensor read 10cm
	Calibrate_Forward(setPoint);

}
//Move forward until one sensor reads setPoint distance
void Calibrate_Forward(int setPoint) 
{
	while (1)
	{
		double leftSensor = ir_FL.getDistance();

		//Distance between robot and setpoint
		//If negative, it means that the robot is too near from the setpoint
		//If positive, it means that the robot is too far from the setpoint
		double distance = leftSensor - setPoint;

		if (distance > 0) //Robot away from setpoint
		{
			motor.Forward(1, false);
		}
		else if (distance < 0) //Robot too 
		{
			motor.Forward(1, true);
		}
		else
			break;
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

	Calibrate_SensorAngle(10);
	
	/*for (int i = 0; i < 4; ++i)
	{
		if (i % 2)
		{
			motor.Turn(-10);
		}
		else
			motor.Turn(10);
		
		delay(1000);

	}*/
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

