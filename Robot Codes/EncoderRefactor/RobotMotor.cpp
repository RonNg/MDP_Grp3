#include "RobotMotor.h"

#define TICK_REFRESH_INTERVAL 1000 //ms
#define TICKS_PER_CM 119 
#define TICKS_PER_REV 2248.86

#define MOTOR_LEFT 1
#define MOTOR_RIGHT 2

#define PRATIO_LEFT 3.17383
#define PRATIO_RIGHT 3.23021

#define INITIAL_POW 220

void RobotMotor::begin()
{
	m1Ticks = 0;
	m2Ticks = 0;

	m1RPM = 0;
	m2RPM = 0;

	m1TargetCounter = 0;
	m2TargetCounter = 0;

	m1Power = 160;
	m2Power = 170;

	lastTime = 0;

	errSum = 0;
	lastErr = 0;

	md.init();
}

void RobotMotor::CalcTicks()
{
	if ((millis() - lastTime) >= TICK_REFRESH_INTERVAL)
	{


		m1RPM = ((m1Ticks*(60000 / TICK_REFRESH_INTERVAL)) / TICKS_PER_REV); //Extrapolate RPM
		m2RPM = ((m2Ticks*(60000 / TICK_REFRESH_INTERVAL)) / TICKS_PER_REV); //100ms * 600ms = 1 minute (interpolate RPM)
		m1Ticks = 0;
		m2Ticks = 0;


		Serial.print(m1RPM);
		Serial.print(" : ");
		Serial.print(m2RPM);
		Serial.println();

		Serial.println ("=== Power ===");
		Serial.print(m1Power);
		Serial.print(", ");
		Serial.print(m2Power);
		Serial.println();
		Serial.println("===========");


		lastTime = millis();
	}




}

double RobotMotor::ComputePID(double consKp, double consKi, double consKd, char direction, double targetRPM, int motorNo)
{
		
	int currentRPM;
	int ratio = 0;

	if (motorNo == MOTOR_LEFT)
	{
		//Serial.println("Motor left");
		ratio = PRATIO_LEFT;
		currentRPM = m1RPM;
	}
	else if (motorNo == MOTOR_RIGHT)
	{
		currentRPM = m2RPM;
		ratio = PRATIO_RIGHT;
		//Serial.println("Motor right");
	}

	//Setpoint - Input(Target Tick)
	double error = targetRPM - currentRPM;

	//Integral
	errSum += error;

	//Derivative
	double dErr = error - lastErr;
	lastErr = error;

	return ((consKp * error) + (consKi * errSum) + (consKd * dErr));
}

bool runOnce = false;
void RobotMotor::ForwardCalibration(int rpm)
{
	/*if (!runOnce)
	{
		md.setSpeeds(280, 280);
		runOnce = true;
		delay(1500);
			
	}*/

	m1Power += ComputePID(0.0002, 0, 0.00005, 'f', rpm, MOTOR_LEFT);
	m2Power += ComputePID(0.0002, 0, 0.00005, 'f', rpm, MOTOR_RIGHT);

	if (m1Power < 0)
		m1Power = 0;

	if (m2Power < 0)
		m2Power = 0;

	md.setSpeeds(m1Power, m2Power);
}

void RobotMotor::Forward(double cm)
{

	int targetTick = cm * TICKS_PER_CM;
	m1TargetCounter = m2TargetCounter = 0;

	Serial.println("Start of forward");

	m1Power = 200;
	m2Power = 200;

	while (m1TargetCounter < targetTick || m2TargetCounter < targetTick)
	{
		CalcTicks();

		Serial.print("Left RPM: ");
		Serial.print(m1RPM);

		Serial.print(", Right RPM: ");
		Serial.print(m2RPM);
		Serial.println();

		m1Power += ComputePID(0.0025, 0, 0.0005, 'f', 40, MOTOR_LEFT);
		m2Power += ComputePID(0.008, 0, 0.0005, 'f', 40, MOTOR_RIGHT);

		if (m1Power < 0)
			m1Power = 0;

		if (m2Power < 0)
			m2Power = 0;

		md.setSpeeds(m1Power, m2Power);
	}

	md.setBrakes(400, 400);
}

void RobotMotor::Turn()
{
	//Diameter of wheel to wheel is 17cm
	//For a 90 deg turn, both wheels must turn the robot 45 degs in each direction
	//By using the formula Pi * 17 * 45/360 
	//We can find that the distance each wheel must travel is 6.67cm

	//Turn 90 deg
	//int targetTick = 6.67 * TICKS_PER_CM;
	int targetTick = 13 * TICKS_PER_CM;
	m1TargetCounter = m2TargetCounter = 0;

	while (m1TargetCounter < targetTick - 90 || m2TargetCounter < targetTick)
	{
		CalcTicks();

		Serial.print("Left RPM: ");
		Serial.print(m1RPM);

		Serial.print(", Right RPM: ");
		Serial.print(m2RPM);
		Serial.println();

		m1Power += ComputePID(0.0020, 0, 0.0005, 'f', 40, true);
		m2Power += ComputePID(0.0025, 0, 0.0005, 'f', 40, false);

		if (m1Power <= 0)
			m1Power = 0;

		//Right turn the right motor must go backwards
		if (m2Power <= 0)
			m2Power = 0;

		md.setSpeeds(m1Power, -m2Power);
	}

	Serial.print(m1Power);
	Serial.print(", ");
	Serial.print(m2Power);

	md.setBrakes(400, 370);
}
