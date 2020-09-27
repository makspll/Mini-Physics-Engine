#pragma once
#include "Precision.h"
#include "Utility.h"

using namespace phy;

real phy::clampAngRad(real rads)
{
	real x = fmod(rads + PI, 2 * PI);
	if (x < 0)
		x += 2 * PI;
	return x - PI;
}

