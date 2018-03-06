#include "RobotMotor.h"


void RobotMotor::begin()
{
	m1Ticks = 0;
	m2Ticks = 0;

	m1RPM = 0;
	m2RPM = 0;

	m1TargetCounter = 0;
	m2TargetCounter = 0;

	m1Power = 150;
	m2Power = 150;

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

	m1Power = m2Power = 100;


	while (m1TargetCounter < targetTick - 60 || m2TargetCounter < targetTick - 60)
	{
		CalcTicks();

		m1Power += ComputePID(0.000048, 0, 0.005, 'f', 100, MOTOR_LEFT);
		m2Power += ComputePID(0.000050, 0, 0.005, 'f', 100, MOTOR_RIGHT);

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
	delay(200);
}

void RobotMotor::Forward10()
{
	int targetTick = 10 * TICKS_PER_CM;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;

	ResetPID();

	//Initial speed
	m1Power = m2Power = 150;

	md.setSpeeds(0, 0);
	md.setBrakes(0, 0);

	while (m1TargetCounter < targetTick - 80 || m2TargetCounter < targetTick - 80)
	{
		CalcTicks();


		m1Power += ComputePID(0.00049, 0, 0.005, 'f', 100, MOTOR_LEFT);
		m2Power += ComputePID(0.000525, 0, 0.003, 'f', 100, MOTOR_RIGHT);

		if (m1Power < 0)
			m1Power = 0;

		if (m2Power < 0)
			m2Power = 0;

		md.setSpeeds(m1Power, m2Power);

	}

	md.setSpeeds(0, 0);
	md.setBrakes(400, 390);

	delay(400);

}
void RobotMotor::Forward30()
{
	int targetTick = 30 * TICKS_PER_CM;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;

	ResetPID();

	//Initial speed
	m1Power = m2Power = 150;

	md.setSpeeds(0, 0);
	md.setBrakes(0, 0);

	while (m1TargetCounter < targetTick - 150 || m2TargetCounter < targetTick - 150)
	{
		CalcTicks();

		m1Power += ComputePID(0.00030, 0, 0.005, 'f', 100, MOTOR_LEFT);
		m2Power += ComputePID(0.000315, 0, 0.003, 'f', 100, MOTOR_RIGHT);

		if (m1Power < 0)
			m1Power = 0;

		if (m2Power < 0)
			m2Power = 0;

		md.setSpeeds(m1Power, m2Power);
	}

	md.setSpeeds(0, 0);
	md.setBrakes(400, 390);

	delay(400);
}
void RobotMotor::Forward50()
{
	int targetTick = 30 * TICKS_PER_CM;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;

	ResetPID();

	//Initial speed
	m1Power = m2Power = 150;

	md.setSpeeds(0, 0);
	md.setBrakes(0, 0);

	while (m1TargetCounter < targetTick - 150 || m2TargetCounter < targetTick - 150)
	{
		CalcTicks();

		m1Power += ComputePID(0.00030, 0, 0.005, 'f', 100, MOTOR_LEFT);
		m2Power += ComputePID(0.000315, 0, 0.003, 'f', 100, MOTOR_RIGHT);

		if (m1Power < 0)
			m1Power = 0;

		if (m2Power < 0)
			m2Power = 0;

		md.setSpeeds(m1Power, m2Power);
	}

	md.setSpeeds(0, 0);
	md.setBrakes(400, 390);

	delay(400);
}

void RobotMotor::ForwardChecklist(int cm)
{
	int targetTick = cm * TICKS_PER_CM;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;

	ResetPID();

	m1Power = m2Power = 150;


	while (m1TargetCounter < targetTick - 60 || m2TargetCounter < targetTick - 60)
	{
		CalcTicks();

		m1Power += ComputePID(0.000047, 0, 0.005, 'f', 100, MOTOR_LEFT);
		m2Power += ComputePID(0.0000505, 0, 0.005, 'f', 100, MOTOR_RIGHT);

		if (m1Power < 0)
			m1Power = 0;

		if (m2Power < 0)
			m2Power = 0;

		md.setSpeeds(m1Power, m2Power);

	}

	md.setSpeeds(0, 0);
	md.setBrakes(400, 400);
	delay(200);

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

		m1Power += ComputePID(0.00048, 0, 0.0005, 'f', 100, MOTOR_LEFT);
		m2Power += ComputePID(0.0005, 0, 0.0003, 'f', 100, MOTOR_RIGHT);


		if (angle > 0) //Positive angle indicates clockwise
			md.setSpeeds(m1Power, -m2Power);
		else
			md.setSpeeds(-m1Power, m2Power);
	}

	md.setSpeeds(0, 0);
	md.setBrakes(400, 400);
}

void RobotMotor::TurnLeft90()
{
	//Convert angle to cm
	float distance = PI * 17.0 * (90.0 / 360.0);
	int targetTick = distance * TICKS_PER_CM;


	//Reset all relevant variables.
	ResetPID(); //Reset PID to prevent errors from growing and growing
	m1Power = m2Power = 150;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;		//For calculating ticks


	md.setBrakes(0, 0);
	md.setSpeeds(0, 0);

	while (m1TargetCounter < targetTick - 90 || m2TargetCounter < targetTick - 50)
	{
		CalcTicks();

		m1Power += ComputePID(0.00047, 0, 0.0005, 'f', 100, MOTOR_LEFT);
		m2Power += ComputePID(0.00051, 0, 0.0005, 'f', 100, MOTOR_RIGHT);

		md.setSpeeds(-m1Power, m2Power);

	}

	md.setSpeeds(0, 0);
	md.setBrakes(400, 400);

	delay(500);
}

void RobotMotor::TurnLeft45()
{
	//Convert angle to cm
	float distance = PI * 17.0 * (45.0 / 360.0);
	int targetTick = distance * TICKS_PER_CM;


	//Reset all relevant variables.
	ResetPID(); //Reset PID to prevent errors from growing and growing
	m1Power = m2Power = 150;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;		//For calculating ticks


	md.setBrakes(0, 0);
	md.setSpeeds(0, 0);

	while (m1TargetCounter < targetTick - 90 || m2TargetCounter < targetTick - 50)
	{
		CalcTicks();

		m1Power += ComputePID(0.00047, 0, 0.0005, 'f', 100, MOTOR_LEFT);
		m2Power += ComputePID(0.00051, 0, 0.0005, 'f', 100, MOTOR_RIGHT);

		md.setSpeeds(-m1Power, m2Power);

	}

	md.setSpeeds(0, 0);
	md.setBrakes(400, 400);

	delay(200);
}

void RobotMotor::TurnRight45()
{
	//Convert angle to cm
	float distance = PI * 17.0 * (45.0 / 360.0);
	int targetTick = distance * TICKS_PER_CM;


	//Reset all relevant variables.
	ResetPID(); //Reset PID to prevent errors from growing and growing
	m1Power = m2Power = 150;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;		//For calculating ticks


	md.setBrakes(0, 0);
	md.setSpeeds(0, 0);

	while (m1TargetCounter < targetTick - 70 || m2TargetCounter < targetTick - 90)
	{
		CalcTicks();

		m1Power += ComputePID(0.00047, 0, 0.0005, 'f', 100, MOTOR_LEFT);
		m2Power += ComputePID(0.00051, 0, 0.0005, 'f', 100, MOTOR_RIGHT);

		md.setSpeeds(m1Power, -m2Power);

	}

	md.setSpeeds(0, 0);
	md.setBrakes(400, 400);

	delay(200);
}

void RobotMotor::TurnRight90()
{
	//Convert angle to cm
	float distance = PI * 17.0 * (90.0 / 360.0);
	int targetTick = distance * TICKS_PER_CM;


	//Reset all relevant variables.
	ResetPID(); //Reset PID to prevent errors from growing and growing
	m1Power = m2Power = 150;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;		//For calculating ticks


	md.setBrakes(0, 0);
	md.setSpeeds(0, 0);

	while (m1TargetCounter < targetTick - 70 || m2TargetCounter < targetTick - 90)
	{
		CalcTicks();

		m1Power += ComputePID(0.00047, 0, 0.0005, 'f', 100, MOTOR_LEFT);
		m2Power += ComputePID(0.00051, 0, 0.0005, 'f', 100, MOTOR_RIGHT);

		md.setSpeeds(m1Power, -m2Power);

	}

	md.setSpeeds(0, 0);
	md.setBrakes(400, 400);

	delay(500);
}

void RobotMotor::Turn180()
{
	//Convert angle to cm
	float distance = PI * 17.0 * (180.0 / 360.0);
	int targetTick = distance * TICKS_PER_CM;


	//Reset all relevant variables.
	ResetPID(); //Reset PID to prevent errors from growing and growing
	m1Power = m2Power = 100;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;		//For calculating ticks


	md.setBrakes(0, 0);
	md.setSpeeds(0, 0);

	while (m1TargetCounter < targetTick - 100 || m2TargetCounter < targetTick - 140)
	{
		CalcTicks();

		m1Power += ComputePID(0.000235, 0, 0.0005, 'f', 100, MOTOR_LEFT);
		m2Power += ComputePID(0.00025, 0, 0.0005, 'f', 100, MOTOR_RIGHT);

		md.setSpeeds(m1Power, -m2Power);

	}

	md.setSpeeds(0, 0);
	md.setBrakes(400, 400);

	delay(500);
}




