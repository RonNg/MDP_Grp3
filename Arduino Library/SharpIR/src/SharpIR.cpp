#include <SharpIR.h>

SharpIR::SharpIR(uint8_t _sensorType, uint8_t _sensorPin)
{
	sensorType = _sensorType;
	pin = _sensorPin;
	lastTime = 0;
}

// Sort an array
void SharpIR::sort(int a[], int size) {
	for (int i = 0; i < (size - 1); i++) {
		bool flag = true;
		for (int o = 0; o < (size - (i + 1)); o++) {
			if (a[o] > a[o + 1]) {
				int t = a[o];
				a[o] = a[o + 1];
				a[o + 1] = t;
				flag = false;
			}
		}
		if (flag) break;
	}
}

double  SharpIR::getDistance()
{
	return this->getDistance(true);
}

double  SharpIR::getDistance(bool avoidBurstRead)
{
	if (!avoidBurstRead) while (millis() <= lastTime + 20) {} //wait for sensor's sampling time


	lastTime = millis();

	int NB_SAMPLE = 50;
	int ir_val[NB_SAMPLE];

	for (int i = 0; i < NB_SAMPLE; i++) 
	{
		// Read analog value
		ir_val[i] = analogRead(pin);
	}

	sort(ir_val, NB_SAMPLE);

	switch (sensorType)
	{
	case GP2YA41SK0F:

		distance = 2076 / (ir_val[NB_SAMPLE / 2] - 11);

		if (distance > 30) return 31;
		else if (distance < 4) return 3;
		else return distance;

		break;

	//Short range sensor
	case GP2Y0A21YK0F:

		//distance = 4800 / (ir_val[NB_SAMPLE / 2] - 20);
		return pow(3027.4 / ir_val[NB_SAMPLE/2], 1.2134);
		//

		//if (distance > 80) return 81;
		////else if (distance < 10) return 9;
		//else if (distance < 8) return 7;
		//else return distance;


		break;

	//Long range sensor
	case GP2Y0A02YK0F:

		return 65 * pow((ir_val[NB_SAMPLE/2] * (5.0 / 1023.0)), -1.10);
		//distance = 10650.08 * pow(ir_val[NB_SAMPLE / 2], -0.935) - 10;
		//distance = 9462 / (ir_val[NB_SAMPLE / 2] - 16.92);

		//if (distance > 150) return 151;
		//if (distance < 20) return 19;
		return distance;
	}
}
