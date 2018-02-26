#pragma once
#include <SharpIR.h>

/*==
2YK Sensors range 20-150cm (Big sensor)

21YK Sensor range 10-80cm (Small Sensor)

==*/

//SharpIR ir_FL(GP2Y0A02YK0F, A0);
//SharpIR ir_FR(GP2Y0A02YK0F, A1);

class RobotSensor
{
public:

	//9 grids on the robot. FRONT_LEFT means the forward most left side sensor.
	//This enum does not account for sensor orientation
	enum SENSOR
	{
		FRONT_LEFT, FRONT_MID, FRONT_RIGHT,
		MIDDLE_LEFT, MIDDLE_MID, MIDDLE_RIGHT,
		BACK_LEFT, BACK_MID, BACK_RIGHT
	};

	void begin();
	int GetDistance(SENSOR sensorEnum);

};

