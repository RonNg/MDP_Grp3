#pragma once
#include <DualVNH5019MotorShield.h>

#define TICK_REFRESH_INTERVAL 500//ms
#define TICKS_PER_CM 118
#define TICKS_PER_REV 2248.86

#define MOTOR_LEFT 1
#define MOTOR_RIGHT 2

#define PRATIO_LEFT 3.17383
#define PRATIO_RIGHT 3.23021


#define INITIAL_POW 220

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
	
	inline DualVNH5019MotorShield GetMotor() {	return md; }
	
	//Char dir is for forward and backward
	//motorNo 1 for Left Motor, 2 for Right Motor
	//
	double ComputePID(double consKp, double consKi, double consKd, char direction, double targetRPM, int motorNo);

	void begin();
	void CalcTicks();

	//Returns true when hit target tick
	void CalibrationForward(int rpm);
	void Forward10(bool reverse);	
	void Forward(double cm, bool reverse);
	void Turn(double angle);


	//Specially calibrated version of Right turn.
	void Turn90(bool turnRight);

};

