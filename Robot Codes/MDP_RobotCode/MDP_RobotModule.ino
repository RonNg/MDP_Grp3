#include <DualVNH5019MotorShield.h>
#include <EnableInterrupt.h>
#include "RobotMotor.h"
#include "RobotSensor.h"
#include <math.h>
#include <SharpIR.h>

RobotMotor motor;

/*==
2YK Sensors range 20-150cm (Big sensor)

21YK Sensor range 10-80cm (Small Sensor)
==*/

//GP2Y0A21YK0F = short 10cm to 80cm
//GP2Y0A2YK0F = long 20cm to 150cm
SharpIR ir_FL(GP2Y0A21YK0F, A0);
SharpIR ir_FR(GP2Y0A21YK0F, A1);
SharpIR ir_FC(GP2Y0A21YK0F, A2); //Front center
SharpIR ir_SF(GP2Y0A21YK0F, A3); //Right Forward
SharpIR ir_SB(GP2Y0A21YK0F, A4); //Right Back
SharpIR ir_LM(GP2Y0A02YK0F, A5);

//------------------Motor Pins ---------------------//
const int M1A = 3;
const int M1B = 5;
const int M2A = 11;
const int M2B = 13;


double round2dp(double value)
{
	double rounded = (int)(value* 100 + .5);
	return (double)rounded/ 100;
}
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

		motor.CalcRPM();
		motor.GetMotor().setBrakes(400, 400);
		delay(1005);
	}

	motor.GetMotor().setBrakes(400, 400);
}

//Makes both sensors aligned with each other i.e. have both sensors measure same distance from the front
void Calibrate_FrontAngle()
{
	double leftSensor;
	double rightSensor;
	for (int i = 0; i < 2; ++i)
	{
		//Align both sensors forward first, regardless whether its far or near
		leftSensor = ir_FL.getDistance();
		rightSensor = ir_FR.getDistance();

		double sensorDiff = abs(leftSensor - rightSensor);

		if (sensorDiff <= 0.01)
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

}

//Calibrates right side to align straight
void Calibrate_SideAngle()
{
	double leftSensor;
	double rightSensor;
	for (int i = 0; i < 2; ++i)
	{
		//Align both sensors forward first, regardless whether its far or near
		leftSensor = ir_SF.getDistance();
		rightSensor = ir_SB.getDistance();

		double sensorDiff = abs(leftSensor - rightSensor);

		if (sensorDiff <= 0.01)
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

}

//Move forward until one sensor reads setPoint distance
void Calibrate_Forward(int setPoint = 11.8)
{

	for (int i = 0; i < 4; ++i)
	{
		double frontSensor = ir_FC.getDistance();

		//Distance between robot and setpoint
		//If negative, it means that the robot is too near from the setpoint
		//If positive, it means that the robot is too far from the setpoint
		double distance = frontSensor - setPoint;

		if (abs(distance) <= 0.005)
			break;

		if (distance > 0) //Robot away from setpoint
		{
			//Absolute distance must be passed in as a negative value will mess up the Forward function
			//to indicate reverse instead, we pass true in the second argument
			motor.CalibrationForward(abs(distance), false);
		}
		else if (distance < 0) //Robot too 
		{
			motor.CalibrationForward(abs(distance), true);
		}
	}
}

void Calibrate_Corner()
{
	Calibrate_Forward();
	Calibrate_FrontAngle();

	//Turn left
	motor.TurnRight90();

	//Calibrate
	Calibrate_Forward();
	Calibrate_FrontAngle();

	//Turn left
	motor.TurnLeft90();

	Calibrate_SideAngle();
}

void Calibrate_Side()
{
	motor.TurnRight90();
	
	Calibrate_Forward();
	Calibrate_FrontAngle();

	motor.TurnLeft90();

	Calibrate_SideAngle();
}


void AutoCalibrate_ForwardDistance()
{
	if (NormalizeShortRange(ir_FC.getDistance()) == 1)
	{
		Calibrate_Forward();
	}
}

void AutoCalibrate_ForwardAngle()
{
	if (NormalizeFrontSides(ir_FL.getDistance()) == 1 && NormalizeFrontSides(ir_FR.getDistance()) == 1)
	{
		Calibrate_FrontAngle();
	}
}


void AutoCalibrate_SideAngle()
{
	if (NormalizeSide(ir_SF.getDistance()) == 1 && NormalizeSide(ir_SB.getDistance()) == 1)
	{
		Calibrate_SideAngle();
	}
}

/*_Obstacle90(int distance)
{
	int travelled = ((distance) / 10);
	travelled -= 5;

	CalibrationTest();

	while (ir_FC.getDistance() > 15)
	{
		motor.Forward10();
		--travelled;
	}



	//Obstacle Avoidance Start
	Calibrate_SideAngle();
	Calibrate_Forward(15);

	motor.Forward10();

	motor.TurnLeft90();

	motor.Forward10();

	motor.Forward10();

	motor.TurnRight90();
	//Calibrate_SideAngle();


	for (int i = 0; i < 4; ++i)
	{
		motor.Forward10();
	}

	motor.TurnRight90();


	motor.Forward10();

	motor.Forward10();

	Calibrate_FrontAngle();

	Calibrate_Forward(15);

	motor.TurnLeft90();

	Calibrate_SideAngle();

	//Obstacle Avoidance End
	while (travelled > 0)
	{
		motor.Forward10();
		--travelled;
	}
}

void Checklist_Obstacle45(int distance)
{
	CalibrationTest();
	int travelled = distance / 10;

	travelled -= 6;


	while (ir_FC.getDistance() > 10)
	{
		motor.Forward10();
		--travelled;
	}

	Calibrate_Forward(16);

	//Diagonal turning left to avoid obstacle
	//Start
	motor.TurnLeft45();
	motor.Forward10();
	motor.Forward10();
	motor.Forward10();
	motor.TurnRight45();

	motor.Forward10();
	motor.Forward10();

	motor.TurnRight45();

	motor.Forward10();
	motor.Forward10();
	motor.Forward10();

	motor.TurnRight45();
	//end

	//Behind the block now, making sure we're still along our intended line by calibrating and using the obstacle for ref
	//Start
	Calibrate_Forward(15);

	motor.TurnRight90();

	Calibrate_Forward(15);

	motor.TurnLeft90();
	motor.TurnLeft90();

	Calibrate_SideAngle();
	//End

	while (travelled > 0)
	{
		motor.Forward10();
		--travelled;
	}



}
*/

int NormalizeShortRange(double shortSensor)
{	
	int dist = -1;
	if (shortSensor > 24 && shortSensor <= 34)
	{
		dist = 3;
	}

	if (shortSensor > 15 && shortSensor <= 24)
	{
		dist = 2;
	}

	if (shortSensor <= 15)
	{
		dist = 1;
	}
		return dist;
}

int NormalizeFrontSides(double sideFrontSensor)
{
	if (sideFrontSensor <= 14)
	{
		return 1;
	}
	return -1;

}

int NormalizeSide(double sideSensor)
{
	if (sideSensor <= 15.5)
	{
		return 1;
	}

	return -1;
}

int NormalizeLong(double longSensor)
{
	int dist = -1;

	if (longSensor > 52.8 && 63)
	{
		dist = 5;
	}
	else if (longSensor > 41 && longSensor <= 52.8)
	{
		dist = 4;
	}
	else if (longSensor > 31.6 && longSensor <= 41)
	{
		dist = 3;
	}
	else if (longSensor > 23.5 && longSensor <= 31.6)
	{
		dist = 2;
	}

	else if (longSensor <= 23.5)
	{
		dist = 1;
	}
	return dist;
}


void GridSensorValues()
{
	//delay(500); //Delay in reading senosr
	int fl = NormalizeFrontSides(ir_FL.getDistance());
	int fc = NormalizeShortRange(ir_FC.getDistance());
	int fr = NormalizeFrontSides(ir_FR.getDistance());

	int lm = NormalizeLong(ir_LM.getDistance());
	
	int rf = NormalizeSide(ir_SF.getDistance());
	int rb = NormalizeSide(ir_SB.getDistance());

	//int rf = ir_SF.getDistance();
	//int rb = ir_SB.getDistance();

	Serial.println("P" + String(fl) + "," + String(fc) + "," + String(fr) + "," + String(rf) + "," + String(rb)  + "," + String(lm) );
	//SENSOR|17|15|17|30|20|20

}

void RawSensorValues()
{
	//delay(500); //Delay in reading senosr
	double  fl = ir_FL.getDistance();
	double  fc = ir_FC.getDistance();
	double  fr = ir_FR.getDistance();

	double lm = ir_LM.getDistance();

	double rf = ir_SF.getDistance();
	double rb = ir_SB.getDistance();



	Serial.println("P" + String(fl) + "," + String(fc) + "," + String(fr) + "," + String(rf) + "," + String(rb) + "," + String(lm));
	//SENSOR|17|15|17|30|20|20
}

void setup()
{
	Serial.begin(9600);

	enableInterrupt(M1A, m1Change, CHANGE);
	enableInterrupt(M1B, m1Change, CHANGE);

	enableInterrupt(M2A, m2Change, CHANGE);
	enableInterrupt(M2B, m2Change, CHANGE);

	motor.begin();

	/*for (int i = 0; i < 3; ++i)
	{
		motor.Forward(10);
	}*/

	//Calibrate_Forward(12.5);
}

String commands;
int currIndex = 0; //Current command index
int commandLength = 0;
bool canCalibrate = false;
bool debugAutoCalibrate = false; //Set to false to disable auto calibration

void loop()
{

	if (Serial.available()) //Checks how many bytes available. sizeof(char) = 1 byte
	{
		while (Serial.available() > 0)
		{
			char temp = Serial.read();
			commands += temp; //Adds the character into the string
		}
	}
	else
	{
		if (commandLength > 0 && canCalibrate && debugAutoCalibrate)
		{	
			AutoCalibrate_ForwardAngle();
			AutoCalibrate_ForwardDistance();
			AutoCalibrate_SideAngle();
			canCalibrate = false;
		}
	}

	//Impt to remove these characters else string length will be wrong
	commands.replace("\n", ""); //Removes newline characters when pressing enter
	commands.replace("\r", ""); //Removes return characters when pressing enter

	for (commandLength = 0; commands[commandLength] != '\0'; ++commandLength);
	//commandLength is array size

	//Serial.println(currIndex);
	while (currIndex < commandLength)
	{
		canCalibrate = true;
		char command = commands[currIndex++]; //Get command from String by treating it as an array

		switch (command)
		{
			//WASD is for movement
		case 'w':
			motor.Forward10();
			Serial.println("PY");
			break;
		case 'W':
			motor.Forward30();
			AutoCalibrate_ForwardDistance();
			Serial.println("PY");
			break;
		case 'q':
			motor.Forward50();
			Serial.println("PY");
			break;
		case 's': //Reverse
			motor.Turn180();
			Serial.println("PY");
			break;
		case 'a': //Turn left 90
			motor.TurnLeft90();
			Serial.println("PY");
			break;
		case 'd': //Turn right 90
			motor.TurnRight90();
			Serial.println("PY");
			break;
			//TFGH is for calibration
		case 't':
			Calibrate_Forward();
			Serial.println("PY");
			break;
		case 'f':
			Calibrate_FrontAngle();
			Serial.print("PY");
			break;
		case 'g':
			Calibrate_Corner();
			Serial.println("PY");
			break;
		case 'h':
			Calibrate_Side();
			Serial.println("PY");
			break;

			//IJL are for sensor readings
		case 'k':
			GridSensorValues();
			break;

		case 'l':
			RawSensorValues();
			break;
		default:
			Serial.println("Error");
			break;
		}
	}
}

