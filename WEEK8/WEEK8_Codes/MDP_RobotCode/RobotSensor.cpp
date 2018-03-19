#include "RobotSensor.h"

void RobotSensor::begin()
{
}

int RobotSensor::GetDistance(SENSOR sensorEnum)
{
	switch (sensorEnum)
	{
	case FRONT_LEFT:
		return 1;
	default:
		return 0;
	}
}
