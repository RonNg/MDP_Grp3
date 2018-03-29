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

//Encoder rising edge tick++
void m1Change()
{
	motor.M1Change();
}

void m2Change()
{
	motor.M2Change();
}

//Makes both sensors aligned with each other i.e. have both sensors measure same distance from the front
void Calibrate_FrontAngle()
{
	double leftSensor;
	double rightSensor;
	for (int i = 0; i < 1; ++i)
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
	for (int i = 0; i < 1; ++i)
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
void Calibrate_Forward(double setPoint = 12.3)
{

	for (int i = 0; i < 2; ++i)
	{
		double frontSensor = ir_FC.getDistance();

		//Distance between robot and setpoint
		//If negative, it means that the robot is too near from the setpoint
		//If positive, it means that the robot is too far from the setpoint
		double distance = frontSensor - setPoint;

		if (abs(distance) <= 0.1)
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
	Calibrate_Forward();

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
	if (NormalizeFrontLeft(ir_FL.getDistance()) == 1 && NormalizeFrontRight(ir_FR.getDistance()) == 1)
	{
		Calibrate_FrontAngle();
	}
}

void AutoCalibrate_SideAngle()
{
	if (NormalizeFrontSide(ir_SF.getDistance()) == 1 && NormalizeBackSide(ir_SB.getDistance()) == 1)
	{
		Calibrate_SideAngle();
	}
}

int NormalizeShortRange(double shortSensor)
{
	int dist = -1;

	/*if (shortSensor > 15 && shortSensor <= 23.9)
	{
		dist = 2;
	}
	else */if (shortSensor <= 14.7)
	{
		dist = 1;
	}
	return dist;
}

int NormalizeFrontLeft(double frontLeftSensor)
{
	int dist = -1;

	/*if (frontLeftSensor > 14 && frontLeftSensor <= 24)
	{
		dist = 2;
	}
	else */if (frontLeftSensor <= 11.28)
	{
		dist = 1;
	}
	return dist;
}

int NormalizeFrontRight(double rightFrontSensor)
{
	int dist = -1;
	/*if (rightFrontSensor > 14 && rightFrontSensor <= 22.9)
	{
		dist = 2;
	}
	else */if (rightFrontSensor <= 11.59)
	{
		dist = 1;
	}
	return dist;
}

int NormalizeFrontSide(double frontSideSensor)
{
	int dist = -1;

	/*if (frontSideSensor > 15.5 && frontSideSensor <= 26)
	{
		dist = 2;
	}
	else*/ if (frontSideSensor <= 14.3)
	{
		dist = 1;
	}

	return dist;
}

int NormalizeBackSide(double backSideSensor)
{
	int dist = -1;

	/*if (backSideSensor > 15.5 && backSideSensor <= 26)
	{
		dist = 2;
	}
	else */if (backSideSensor <= 13.85)
	{
		dist = 1;
	}

	return dist;
}

int NormalizeLong(double longSensor)
{
	int dist = -1;

	if (longSensor > 43.85 && longSensor <= 51.91)
	{
		dist = 4;
	}
	else if (longSensor > 32.6 && longSensor <= 43.85)
	{
		dist = 3;
	}
	else if (longSensor > 23.5 && longSensor <= 32.6)
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
	//delay(100); //Delay in reading senosr
	int fl = NormalizeFrontLeft(ir_FL.getDistance());
	int fc = NormalizeShortRange(ir_FC.getDistance());
	int fr = NormalizeFrontRight(ir_FR.getDistance());

	int lm = NormalizeLong(ir_LM.getDistance());

	int rf = NormalizeFrontSide(ir_SF.getDistance());
	int rb = NormalizeBackSide(ir_SB.getDistance());

	//int rf = ir_SF.getDistance();
	//int rb = ir_SB.getDistance();

	Serial.println("P" + String(fl) + "," + String(fc) + "," + String(fr) + "," + String(rf) + "," + String(rb) + "," + String(lm));
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

	//Changes ADC prescaler to 32, faster analogRead
	ADCSRA &= ~(bit(ADPS0) | bit(ADPS1) | bit(ADPS2)); // clear prescaler bits

	//ADCSRA |= bit(ADPS0) | bit(ADPS2);                 //  32 
	ADCSRA |= bit(ADPS0) | bit(ADPS1) | bit(ADPS2);   // 128

	enableInterrupt(M1A, m1Change, CHANGE);
	enableInterrupt(M1B, m1Change, CHANGE);

	enableInterrupt(M2A, m2Change, CHANGE);
	enableInterrupt(M2B, m2Change, CHANGE);

	motor.begin();

}

String commands;
int currIndex = 0; //Current command index
int commandLength = 0;
bool canCalibrate = false;
bool debugAutoCalibrate = true; //Set to false to disable auto calibration
bool exploration = true;

void FlushBuffer()
{
	commands.replace(commands, "");
	commandLength = 0;
	currIndex = 0;
}

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

	if ((commandLength - 1) - currIndex >= 2) //Total commands sent is more than 2 i.e. fastest path
	{
		exploration = false;
	}
	else
	{
		exploration = true;
	}

	//Serial.println(currIndex);
	while (currIndex < commandLength)
	{
		canCalibrate = true;
		char command = commands[currIndex++]; //Get command from String by treating it as an array

		if (isDigit(command) && command != '1')
		{

			int forwardDist = (int)(command - 48); //Convert char to int 
			if (forwardDist == 0)
			{
				forwardDist = 10;
			}

			if (forwardDist < 4)
			{
				motor.ForwardShort(forwardDist * 10);
			}
			else
			{
				motor.Forward(forwardDist * 10);
			}
			GridSensorValues();
		}
		else
		{
			switch (command)
			{
				//WASD is for movement
			case '1':
			case 'w':
				motor.Forward10();
				GridSensorValues();
				break;
			//case '5':
				//motor.Forward50();
				//GridSensorValues();
				//break;
			case '0':
				motor.Forward(100);
				GridSensorValues();
				break;
			case '!':
				motor.Forward(110);
				GridSensorValues();
				break;
			case '@':
				motor.Forward(120);
				GridSensorValues();
				break;
			case '#':
				motor.Forward(130);
				GridSensorValues();
				break;
			case '$':
				motor.Forward(140);
				GridSensorValues();
				break;
			case '%':
				motor.Forward(150);
				GridSensorValues();
				break;
			case '^':
				motor.Forward(160);
				GridSensorValues();
				break;
			case '&':
				motor.Forward(170);
				GridSensorValues();
				break;
			case 's': //Reverse
				motor.Turn180();
				GridSensorValues();
				break;
			case 'a': //Turn left 90
				motor.TurnLeft90();
				GridSensorValues();
				break;
			case 'd': //Turn right 90
				motor.TurnRight90();
				GridSensorValues();
				break;
				//TFGH is for calibration
			case 't':
				Calibrate_Forward();
				Serial.println("PY");
				break;
			case 'f':
				Calibrate_FrontAngle();
				Serial.println("PY");
				break;
			case 'g':
				Calibrate_Corner();
				Serial.println("PY");
				break;
			case 'h':
				Calibrate_Side();
				Serial.println("PY");
				break;
			case 'o':
				FlushBuffer();
				Serial.println("PY");
				break;
				//IJL are for sensor readings
			case 'k':
				GridSensorValues();
				break;
			case 'l':
				RawSensorValues();
				break;
			case 'i':
				Serial.println(analogRead(A2));
				break;
			case '[':
				Serial.println(analogRead(A0)); //Left sensor read
				break;
			case ']':
				Serial.println(analogRead(A1)); //Left sensor read
				break;
			case ';':
				debugAutoCalibrate = !debugAutoCalibrate;
				Serial.println(debugAutoCalibrate);
				break;
			default:
				Serial.println("Error");
				break;
			}
		}
		if (!exploration)
		{
			AutoCalibrate_ForwardAngle();
			AutoCalibrate_ForwardDistance();
			AutoCalibrate_SideAngle();
		}
	}
}