#include "RobotMotor.h"

//SharpIR brakeSensor(GP2Y0A21YK0F, A2);



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

	while (m1TargetCounter < targetTick - 80 || m2TargetCounter < targetTick - 80)
	{
		CalcRPM();
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
	m1Power = m2Power = 250;

	md.setBrakes(0, 0);
	md.setSpeeds(m1Power, m2Power);


	while (m1TargetCounter < targetTick - 125 || m2TargetCounter < targetTick - 125)
	{

		//Analog ADC read. 345 is the stop distance in 2^10 bits integer value
		if (analogRead(A2) >= 432 || analogRead(A0) >= 535 || analogRead(A1) >= 535)
		{
			break;
		}
		CalcRPM();

		m1Power += ComputePID(0.0049, 0, 0.005, 100, MOTOR_LEFT);
		m2Power += ComputePID(0.0055, 0.00000025, 0.005, 100, MOTOR_RIGHT);

		if (m1Power < 0)
			m1Power = 0;

		if (m2Power < 0)
			m2Power = 0;


		md.setSpeeds(m1Power, m2Power);

		delay(1);
	}

	md.setSpeeds(0, 0);
	md.setBrakes(300, 300);

	delay(200);
}
void RobotMotor::ForwardShort(double cm)
{
	int targetTick = cm * TICKS_PER_CM;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;

	ResetPID();

	//Initial speed
	m1Power = m2Power = 250;

	md.setBrakes(0, 0);
	md.setSpeeds(m1Power, m2Power);


	while (m1TargetCounter < targetTick - 125 || m2TargetCounter < targetTick - 125)
	{

		//Analog ADC read. 345 is the stop distance in 2^10 bits integer value
		if (analogRead(A2) >= 432 || analogRead(A0) >= 535 || analogRead(A1) >= 535)
		{
			break;
		}
		CalcRPM();

		m1Power += ComputePID(0.00475, 0, 0.003, 100, MOTOR_LEFT);
		m2Power += ComputePID(0.00557, 0.00000025, 0.003, 100, MOTOR_RIGHT);

		if (m1Power < 0)
			m1Power = 0;

		if (m2Power < 0)
			m2Power = 0;


		md.setSpeeds(m1Power, m2Power);

		delay(1);
	}

	md.setSpeeds(0, 0);
	md.setBrakes(300, 300);

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
	m1Power = m2Power = 200;

	md.setBrakes(0, 0);
	//md.setSpeeds(0, m2Power);
	//delay(20);
	//md.setSpeeds(m1Power, 0);
	md.setSpeeds(m1Power, m2Power);

	while (m1TargetCounter < targetTick - 140 || m2TargetCounter < targetTick - 140)
	{
		//Analog ADC read. 345 is the stop distance in 2^10 bits integer value
		if (analogRead(A2) >= 432 || analogRead(A0) >= 535 || analogRead(A1) >= 535)
		{
			break;
		}

		CalcRPM();
		//m1Power += ComputePID(0.03, 0, 0.0009, 80, MOTOR_LEFT);
		//m2Power += ComputePID(0.032, 0, 0.0009, 80, MOTOR_RIGHT);
		m1Power += ComputePID(0.00480, 0, 0.01, 80, MOTOR_LEFT);
		m2Power += ComputePID(0.00535, 0, 0.01, 80, MOTOR_RIGHT);

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
	md.setBrakes(330, 300);

	delay(500);
}
void RobotMotor::Forward50()
{
	int targetTick = 50 * TICKS_PER_CM;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;

	ResetPID();

	//Initial speed
	m1Power = m2Power = 250;

	md.setBrakes(0, 0);
	md.setSpeeds(m1Power, m2Power);


	while (m1TargetCounter < targetTick - 125 || m2TargetCounter < targetTick - 125)
	{

		//Analog ADC read. 345 is the stop distance in 2^10 bits integer value
		if (analogRead(A2) >= 432 || analogRead(A0) >= 535 || analogRead(A1) >= 535)
		{
			break;
		}
		CalcRPM();

		m1Power += ComputePID(0.0049, 0, 0.005, 100, MOTOR_LEFT);
		m2Power += ComputePID(0.00545, 0.0000001, 0.005, 100, MOTOR_RIGHT);

		if (m1Power < 0)
			m1Power = 0;

		if (m2Power < 0)
			m2Power = 0;


		md.setSpeeds(m1Power, m2Power);

		delay(1);
	}

	md.setSpeeds(0, 0);
	md.setBrakes(380, 380);

	delay(200);
}
void RobotMotor::Forward70()
{
	int targetTick = 50 * TICKS_PER_CM;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;

	ResetPID();

	//Initial speed
	m1Power = m2Power = 250;

	md.setBrakes(0, 0);
	md.setSpeeds(m1Power - 70, m2Power);


	while (m1TargetCounter < targetTick - 125 || m2TargetCounter < targetTick - 125)
	{

		//Analog ADC read. 345 is the stop distance in 2^10 bits integer value
		if (analogRead(A2) >= 432 || analogRead(A0) >= 535 || analogRead(A1) >= 535)
		{
			break;
		}
		CalcRPM();

		m1Power += ComputePID(0.005, 0, 0.0009, 100, MOTOR_LEFT);
		m2Power += ComputePID(0.006, 0, 0.0009, 100, MOTOR_RIGHT);

		if (m1Power < 0)
			m1Power = 0;

		if (m2Power < 0)
			m2Power = 0;


		md.setSpeeds(m1Power, m2Power);

		delay(1);
	}

	md.setSpeeds(0, 0);
	md.setBrakes(380, 380);

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
	md.setSpeeds(-(m1Power-250), m2Power);
	md.setBrakes(0, 0);

	while (m1TargetCounter < targetTick - 36 || m2TargetCounter < targetTick - 36)
	{
		CalcRPM();

		m1Power += ComputePID(0.0035, 0, 0.0009, 70, MOTOR_LEFT);
		m2Power += ComputePID(0.0047, 0, 0.0009, 70, MOTOR_RIGHT);

		md.setSpeeds(-m1Power, m2Power);

		delay(1);
	}

	md.setSpeeds(0, 0);
	md.setBrakes(380, 380);

	delay(350);
}


void RobotMotor::TurnRight90()
{
	//Convert angle to cm
	float distance = PI * 17.0 * (90.0 / 360.0);
	int targetTick = distance * TICKS_PER_CM;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;		//For calculating ticks

	ResetPID();


	m1Power = m2Power = 250;
	md.setSpeeds(m1Power-150, -m2Power);
	md.setBrakes(0, 0);

	while (m1TargetCounter < targetTick - 42 || m2TargetCounter < targetTick - 42)
	{
		CalcRPM();

		m1Power += ComputePID(0.0048, 0, 0.0009, 70, MOTOR_LEFT);
		m2Power += ComputePID(0.0050 ,0, 0.0009, 70, MOTOR_RIGHT);

		md.setSpeeds(m1Power, -m2Power);

		delay(1);
	}

	md.setSpeeds(0, 0);
	md.setBrakes(380, 380);

	delay(350);
}

void RobotMotor::Turn180()
{
	//Convert angle to cm
	float distance = PI * 17.0 * (180.0 / 360.0);
	int targetTick = distance * TICKS_PER_CM;
	m1TargetCounter = m2TargetCounter = 0;
	m1Ticks = m2Ticks = 0;
	lastTime = 0;		//For calculating ticks

	ResetPID();


	m1Power = m2Power = 250;
	md.setSpeeds(m1Power - 150, -m2Power);
	md.setBrakes(0, 0);

	while (m1TargetCounter < targetTick + 20 || m2TargetCounter < targetTick + 20 )
	{
		CalcRPM();

		m1Power += ComputePID(0.0048, 0, 0.0009, 70, MOTOR_LEFT);
		m2Power += ComputePID(0.0050, 0, 0.0009, 70, MOTOR_RIGHT);

		md.setSpeeds(m1Power, -m2Power);

		delay(1);
	}

	md.setSpeeds(0, 0);
	md.setBrakes(380, 380);

	delay(350);
}