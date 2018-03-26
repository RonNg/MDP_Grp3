#pragma once
#include <DualVNH5019MotorShield.h>
#include "SharpIR.h"
#include <Arduino.h>

#define TICK_REFRESH_INTERVAL 200//ms
#define TICKS_PER_CM 118
#define TICKS_PER_REV 2248.86

#define MOTOR_LEFT 1
#define MOTOR_RIGHT 2

#define PRATIO_LEFT 3.17383 //1 power = 3.17 RPM for left motor
#define PRATIO_RIGHT 3.23021 //1 power = 3.23 RPM for right motor

/*
	NEED TO RECALIBRATE FORWARD 10, ROTATE LEFT 90 AND ROTATE RIGHT 90 AND TURN 180
*/

class RobotMotor
{
private:

	volatile double m1Ticks;
	volatile double m2Ticks;

	volatile double m1TargetCounter; //For counting to target ticks bnased on distance
	volatile double m2TargetCounter;

	double m1RPM;
	double m2RPM;

	double m1Power;
	double m2Power;

	//For PID computation
	double errSum;
	double lastErr;

	unsigned long lastTime;

	DualVNH5019MotorShield md;

public:
	inline void M1Change() { m1Ticks++; m1TargetCounter++; }
	inline void M2Change() { m2Ticks++; m2TargetCounter++; }
	inline void ResetPID() { errSum = 0; lastErr = 0; }
	inline int getM1Tick() {
		return m1Ticks;
	}

	inline int getM2Tick() {
		return m2Ticks;
	}
	
	inline DualVNH5019MotorShield GetMotor() {	return md; }
	
	//Char dir is for forward and backward
	//motorNo 1 for Left Motor, 2 for Right Motor
	//
	double ComputePID(double consKp, double consKi, double consKd, double targetRPM, int motor);

	void begin();
	void CalcRPM();

	//Returns true when hit target tick
	void CalibrationForward(double cm, bool reverse);
	
	void Forward(double cm);
	void Forward10();
	void Forward50();
	void Forward70();

	void Turn(double angle);

	//Specially calibrated version of moving forward

	//Specially calibrated version of Left and right 90 degree turn.
	void TurnRight90();
	void TurnLeft90();

	void Turn180();
};

