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

void RobotMotor::CalcRPM()
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

double RobotMotor::ComputePID(double consKp, double consKi, double consKd, double targetRPM, int motor)
{
	int currentRPM;
	
	if (motor == MOTOR_LEFT)
		currentRPM = m1RPM;
	else
		currentRPM = m2RPM;

	//Setpoint - Input(Target Tick)
	double error = targetRPM - currentRPM;

	//Integral
	errSum += error;

	//Derivative
	double dErr = error - lastErr;
	lastErr = error;

	return ((consKp * error) + (consKi * errSum) + (consKd * dErr));
}

void RobotMotor::CalibrationForward(double  cm, bool reverse)
{
	int targetTick = cm * TICKS_PER_CM;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;

	ResetPID();

	//Initial speed
	m1Power = m2Power = 200;

	md.setSpeeds(m1Power, m2Power);
	md.setBrakes(0, 0);

	while (m1TargetCounter < targetTick - 10 || m2TargetCounter < targetTick - 10)
	{
		CalcRPM();
		/*m1Power += ComputePID(0.000475, 0, 0.005, 'f', 100, MOTOR_LEFT);
		m2Power += ComputePID(0.00051, 0, 0.003, 'f', 100, MOTOR_RIGHT);*/
		m1Power += ComputePID(0.000540, 0, 0.005, 80, MOTOR_LEFT);
		m2Power += ComputePID(0.000535, 0, 0.005, 80, MOTOR_RIGHT);

		if (m1Power < 0)
			m1Power = 0;

		if (m2Power < 0)
			m2Power = 0;

		if(reverse)
			md.setSpeeds(-m1Power, -m2Power);
		else
			md.setSpeeds(m1Power, m2Power);

		delay(1);
	}

	md.setSpeeds(0, 0);
	md.setBrakes(400, 400);

	delay(100);
}

void RobotMotor::Forward(double cm)
{
	int targetTick = cm * TICKS_PER_CM;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;

	ResetPID();

	//Initial speed
	m1Power = m2Power = 300;

	md.setBrakes(0, 0);
	md.setSpeeds(m1Power, m2Power);


	while (m1TargetCounter < targetTick - 90 || m2TargetCounter < targetTick - 90)
	{
		CalcRPM();
		m1Power += ComputePID(0.000540, 0, 0.005, 100, MOTOR_LEFT);
		m2Power += ComputePID(0.000535, 0, 0.005, 100, MOTOR_RIGHT);

		if (m1Power < 0)
			m1Power = 0;

		if (m2Power < 0)
			m2Power = 0;

		md.setSpeeds(m1Power, m2Power);

		/*Serial.print(m1RPM);
		Serial.print(",");
		Serial.println(m2RPM);*/

		delay(1); //Slows down PID compute 
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
	m1Power = m2Power = 250;

	md.setBrakes(0, 0);
	md.setSpeeds(m1Power, m2Power);


	while (m1TargetCounter < targetTick - 70 || m2TargetCounter < targetTick - 70)
	{
		CalcRPM();
		m1Power += ComputePID(0.000350, 0, 0.005, 100, MOTOR_LEFT);
		m2Power += ComputePID(0.000570, 0, 0.005, 100, MOTOR_RIGHT);

		if (m1Power < 0)
			m1Power = 0;

		if (m2Power < 0)
			m2Power = 0;

		md.setSpeeds(m1Power, m2Power);

		/*Serial.print(m1RPM);
		Serial.print(",");
		Serial.println(m2RPM);*/

		delay(1); //Slows down PID compute 
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
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;		//For calculating ticks

	ResetPID();
	

	m1Power = m2Power = 250;

	md.setSpeeds(m1Power, m2Power);
	md.setBrakes(0, 0);

	while (m1TargetCounter < targetTick - 50 || m2TargetCounter < targetTick - 50)
	{
		CalcRPM();

		m1Power += ComputePID(0.000555, 0, 0.005, 100, MOTOR_LEFT);
		m2Power += ComputePID(0.000535, 0, 0.005, 100, MOTOR_RIGHT);


		if (angle > 0) //Positive angle indicates clockwise
			md.setSpeeds(m1Power, -m2Power);
		else
			md.setSpeeds(-m1Power, m2Power);

		delay(1);
	}

	md.setSpeeds(0, 0);
	md.setBrakes(400, 400);
}

void RobotMotor::TurnLeft90()
{
	//Convert angle to cm
	float distance = PI * 17.0 * (90.0 / 360.0);
	int targetTick = distance * TICKS_PER_CM;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;		//For calculating ticks

	ResetPID();


	m1Power = m2Power = 250;

	md.setSpeeds(m1Power, m2Power);
	md.setBrakes(0, 0);

	while (m1TargetCounter < targetTick - 50 || m2TargetCounter < targetTick - 50)
	{
		CalcRPM();

		m1Power += ComputePID(0.000550, 0, 0.005, 90, MOTOR_LEFT);
		m2Power += ComputePID(0.000535, 0, 0.005, 90, MOTOR_RIGHT);

		md.setSpeeds(-m1Power, m2Power);

		delay(1);
	}

	md.setSpeeds(0, 0);
	md.setBrakes(400, 400);

	delay(300);
}

void RobotMotor::TurnRight90()
{
	//Convert angle to cm
	float distance = PI * 17.0 * (90.0 / 360.0);
	int targetTick = distance * TICKS_PER_CM;


	//Reset all relevant variables.
	ResetPID(); //Reset PID to prevent errors from growing and growing
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;		//For calculating ticks

	m1Power = m2Power = 250;
	md.setSpeeds(m1Power, -m2Power);
	md.setBrakes(0, 0);

	while (m1TargetCounter < targetTick - 80 || m2TargetCounter < targetTick - 80)
	{
		CalcRPM();

		m1Power += ComputePID(0.000530, 0, 0.005, 90, MOTOR_LEFT);
		m2Power += ComputePID(0.000530, 0, 0.005, 90, MOTOR_RIGHT);

		md.setSpeeds(m1Power, -m2Power);

		delay(1);
	}

	md.setSpeeds(0, 0);
	md.setBrakes(400, 400);

	delay(300);
}

void RobotMotor::Turn180()
{
//Convert angle to cm
	float distance = PI * 17.0 * (180.0 / 360.0);
	int targetTick = distance * TICKS_PER_CM;


	//Reset all relevant variables.
	ResetPID(); //Reset PID to prevent errors from growing and growing
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;		//For calculating ticks

	m1Power = m2Power = 250;
	md.setSpeeds(m1Power, -m2Power);
	md.setBrakes(0, 0);

	while (m1TargetCounter < targetTick - 30 || m2TargetCounter < targetTick - 30)
	{
		CalcRPM();

		m1Power += ComputePID(0.000550, 0, 0.005, 90, MOTOR_LEFT);
		m2Power += ComputePID(0.000535, 0, 0.005, 90, MOTOR_RIGHT);

		md.setSpeeds(-m1Power, m2Power);

		delay(1);
	}

	md.setSpeeds(0, 0);
	md.setBrakes(400, 400);

	delay(400);
}



