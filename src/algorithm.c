/*
 * algorithm.c
 *
 *  Created on: 11 lip 2016
 *      Author: kormo
 */
#include "algorithm.h"
#include "resources.h"
#include <math.h>

/*
Zak�adaj�c, �e mamy warto�� x z przedzia�u [x_min, x_max]
i chcemy j� przenie�� do [y_min, y_max], mo�na skorzysta� ze wzoru:

*/
int rangeScaleLinear
(uint32_t x,
		uint32_t x_min,
		uint32_t x_max,
		uint32_t y_min,
		uint32_t y_max)

{
	return (uint32_t) y_min+(x-x_min)*(y_max-y_min)/(x_max-x_min);
};

// f(v)= f0*2^ (v-v0/v0)

double rangeScaleVoltPerOclave
(double v,
		double v0,
		double f0
		)

{
	double f;
	double power;
	uint8_t pint;
	uint8_t powindex;
	double pfrac;

	power = ((v-v0)/v0);

	pint=(int)floor(power);
	pfrac=power-pint;

	powindex = (int)(pint/0.001);

	//f=f0 * pow(2.0,power);
	//PowerOf2_16bit


	f=f0*PowerOf2_16bit[powindex];

	//*TODO dodac obslug� pint>0
	//*TODO dodac obsluge pint <0

	return f;

};
