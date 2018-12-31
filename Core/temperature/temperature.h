#ifndef TEMPERATURE_TEMPERATURE_H_
#define TEMPERATURE_TEMPERATURE_H_

#include "math.h"
#include "stm32l4xx_hal.h"

#define SERIESRESISTOR 10000      // thermistor voltage divider static resistance
#define THERMISTORNOMINAL 10000   // thermistor resistance @ 25C
#define BCOEFFICIENT 3950         // thermistor beta coefficient
#define TEMPERATURENOMINAL 25     // temp. for nominal resistance (almost always 25C)
#define ADCResolution 4096        //ADC resolution (ex. 12 bits is 2^12=4096

float CalculateSteinhart(uint32_t measuredVolts);

#endif /* TEMPERATURE_TEMPERATURE_H_ */
