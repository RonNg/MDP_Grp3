#pragma once
#include <DualVNH5019MotorShield.h>

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

	void ForwardCalibration(int rpm);
	void Forward(double cm);	
	void Turn();

};

