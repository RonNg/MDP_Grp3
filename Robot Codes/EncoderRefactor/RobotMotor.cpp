#include "RobotMotor.h"


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

void RobotMotor::CalibrationForward(int rpm)
{

	
	m1Power += ComputePID(0.0002, 0, 0.00005, 'f', rpm, MOTOR_LEFT);
	m2Power += ComputePID(0.0002, 0, 0.00005, 'f', rpm, MOTOR_RIGHT);

	if (m1Power < 0)
		m1Power = 0;

	if (m2Power < 0)
		m2Power = 0;

	md.setSpeeds(m1Power, m2Power);
}

void RobotMotor::Forward(double cm, bool reverse)
{
	Serial.println(cm);
	int targetTick = cm * TICKS_PER_CM;
	m1TargetCounter = m2TargetCounter = 0;

	Serial.println("Forward");

	m1Power = 200;
	m2Power = 200;

	while (m1TargetCounter < targetTick - 50  || m2TargetCounter < targetTick - 50)
	{
		CalcTicks();

		Serial.print("Left RPM: ");
		Serial.print(m1RPM);

		Serial.print(", Right RPM: ");
		Serial.print(m2RPM);
		Serial.println();

		m1Power += ComputePID(0.0025, 0, 0.0005, 'f', 80, MOTOR_LEFT);
		m2Power += ComputePID(0.0026, 0, 0.0005, 'f', 80, MOTOR_RIGHT);

		if (m1Power < 0)
			m1Power = 0;

		if (m2Power < 0)
			m2Power = 0;

		if(!reverse)
			md.setSpeeds(m1Power, m2Power);
		else
			md.setSpeeds(-m1Power, -m2Power);
	}

	md.setBrakes(400, 400);
}

void RobotMotor::Turn(double angle)
{
	//Wheel to wheel is 17cm i.e. Diameter
	//Pi * d * TurnAngle/360 = Distant to travel for wheel (Formula for length of arc)
	//1cm = 119 ticks

	//For a 90 deg turn, both wheels must turn the robot 90 degs in each direction
	//By using the formula Pi * 17 * 45/360 
	//We can find that the distance each wheel must travel is 6.67cm


	//Turn 90 deg
	//int targetTick = 6.67 * TICKS_PER_CM;

	//Convert angle to cm
	float distance = PI * 17.0 * abs(angle) / 360.0;
	
	m1Power = m2Power = 0;
	int targetTick = distance * TICKS_PER_CM;
	m1TargetCounter = m2TargetCounter = 0;

	while (m1TargetCounter < targetTick - 100 || m2TargetCounter < targetTick - 80)
	{
		CalcTicks();
		  
		//Serial.print("Left RPM: ");
		//Serial.print(m1RPM);

		//Serial.print(", Right RPM: ");
		//Serial.print(m2RPM);
		//Serial.println();

		


		m1Power += ComputePID(0.0025, 0, 0.0005, 'f', 80, MOTOR_LEFT);
		m2Power += ComputePID(0.0025, 0, 0.0005, 'f', 80, MOTOR_RIGHT);

		
		if (angle > 0) //Positive angle indicates clockwise
			md.setSpeeds(m1Power, -m2Power);
		else
			md.setSpeeds(-m1Power, m2Power);
	}

	Serial.print(m1Power);
	Serial.print(", ");
	Serial.print(m2Power);

	md.setBrakes(390, 390);

	
}
