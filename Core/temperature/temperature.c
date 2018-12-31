#include "temperature.h"

float CalculateSteinhart(uint32_t measuredVolts)
{
	float steinhart, resistance;

	// convert the value to resistance
	resistance = (((float)4096 / 	(float)measuredVolts) - 1) * (float)SERIESRESISTOR;

	//Thermistor resistance is  related to temperature in degrees Kelvin by the following formula:
	//1/T= A + B*ln(R/Rt) + C*ln(R/Rt)2 + D*ln(R/Rt)3
	steinhart = resistance / THERMISTORNOMINAL;     // (R/Ro)
	steinhart = log(steinhart);                  // ln(R/Ro)
	steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
	steinhart += 1 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
	steinhart = 1 / steinhart;                 // Invert
	steinhart -= 273.15f;                         // convert from K to C

	return steinhart;
}
