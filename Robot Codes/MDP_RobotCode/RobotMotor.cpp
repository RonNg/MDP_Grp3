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
	int targetTick = cm * TICKS_PER_CM;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;

	ResetPID();


	md.setBrakes(0, 0);
	md.setSpeeds(0, 0);

	for (int i = 0; i < 4; ++i)
	{
		md.setSpeeds(i * 50, i * 50);
	}

	m1Power = m2Power = 0;
	

	while (m1TargetCounter < targetTick - 80 || m2TargetCounter < targetTick - 50)
	{
		CalcTicks();

		Serial.print("Left RPM: ");
		Serial.print(m1RPM);

		Serial.print(", Right RPM: ");
		Serial.print(m2RPM);
		Serial.println();

		m1Power += ComputePID(0.049, 0, 0.005, 'f', 100, MOTOR_LEFT);
		m2Power += ComputePID(0.05, 0, 0.005, 'f', 100, MOTOR_RIGHT);

		if (m1Power < 0)
			m1Power = 0;

		if (m2Power < 0)
			m2Power = 0;

		if (!reverse)
			md.setSpeeds(m1Power, m2Power);
		else
			md.setSpeeds(-m1Power, -m2Power);

	}

	md.setSpeeds(0, 0);
	md.setBrakes(400, 400);
}

void RobotMotor::Forward10(bool reverse)
{
	int targetTick = 10 * TICKS_PER_CM;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;

	ResetPID();


	md.setBrakes(0, 0);
	md.setSpeeds(0, 0);

	/*for (int i = 0; i < 3; ++i)
	{
	md.setSpeeds(i * 50, i * 50);
	delay(10);
	}
	*/
	m1Power = m2Power = 0;

	while (m1TargetCounter < targetTick - 80 || m2TargetCounter < targetTick - 50)
	{
		CalcTicks();

		Serial.print("Left RPM: ");
		Serial.print(m1RPM);

		Serial.print(", Right RPM: ");
		Serial.print(m2RPM);
		Serial.println();

		m1Power += ComputePID(0.05, 0, 0.005, 'f', 100, MOTOR_LEFT);
		m2Power += ComputePID(0.051, 0, 0.003, 'f', 100, MOTOR_RIGHT);

		if (m1Power < 0)
			m1Power = 0;

		if (m2Power < 0)
			m2Power = 0;

		if (!reverse)
			md.setSpeeds(m1Power, m2Power);
		else
			md.setSpeeds(-m1Power, -m2Power);

	}

	md.setSpeeds(0, 0);
	md.setBrakes(400, 400);
	
}

void RobotMotor::Turn(double angle)
{
	//Convert angle to cm
	float distance = PI * 17.0 * (abs(angle) / 360.0);
	int targetTick = distance * TICKS_PER_CM;

	//ResetPID();
	
	m1Power = m2Power = 0;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;		//For calculating ticks



	while (m1TargetCounter < targetTick - 50 || m2TargetCounter < targetTick - 50)
	{
		CalcTicks();

		m1Power += ComputePID(0.0005, 0, 0.0005, 'f', 100, MOTOR_LEFT);
		m2Power += ComputePID(0.0005, 0, 0.0003, 'f', 100, MOTOR_RIGHT);

		
		if (angle > 0) //Positive angle indicates clockwise
			md.setSpeeds(m1Power, -m2Power);
		else
			md.setSpeeds(-m1Power, m2Power);
	}

	Serial.print(m1Power);
	Serial.print(", ");
	Serial.print(m2Power);

	md.setBrakes(400, 400);
}


void RobotMotor::TurnLeft90()
{
	//Convert angle to cm
	float distance = PI * 17.0 * (90.0 / 360.0);
	int targetTick = distance * TICKS_PER_CM;


	//Reset all relevant variables.
	ResetPID(); //Reset PID to prevent errors from growing and growing
	m1Power = m2Power = 0;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;		//For calculating ticks


	md.setBrakes(0, 0);
	md.setSpeeds(0, 0);

	while (m1TargetCounter < targetTick - 50 || m2TargetCounter < targetTick - 50)
	{
		CalcTicks();

		m1Power += ComputePID(0.0005, 0, 0.0005, 'f', 100, MOTOR_LEFT);
		m2Power += ComputePID(0.000515, 0, 0.0005, 'f', 100, MOTOR_RIGHT);

		md.setSpeeds(-m1Power, m2Power);

	}

	md.setSpeeds(0, 0);
	md.setBrakes(400, 400);
}

void RobotMotor::TurnRight90()
{
	//Convert angle to cm
	float distance = PI * 17.0 * (90.0/360.0);
	int targetTick = distance * TICKS_PER_CM;


	//Reset all relevant variables.
	ResetPID(); //Reset PID to prevent errors from growing and growing
	m1Power = m2Power = 0;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;	
	lastTime = 0;		//For calculating ticks
	

	md.setBrakes(0, 0);
	md.setSpeeds(0, 0);
	
	while (m1TargetCounter < targetTick - 50 || m2TargetCounter < targetTick - 50)
	{
		CalcTicks();

		m1Power += ComputePID(0.0004, 0, 0.0005, 'f', 100, MOTOR_LEFT);
		m2Power += ComputePID(0.0005, 0, 0.0005, 'f', 100, MOTOR_RIGHT);

		md.setSpeeds(m1Power, -m2Power);
	
	}

	md.setSpeeds(0, 0);
	md.setBrakes(400, 400);
}


