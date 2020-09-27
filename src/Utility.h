#pragma once
#include "Precision.h"

// global functions for utility


namespace phy {
	//constraint the value to [-180,180) e.g. 180 radians doesn't exist
	real clampAngRad(phy::real rads);

	template <class T>
	T minimum(T one, T two);

	template <class T>
	T maximum(T one, T two);


}

template <class T>
T phy::minimum(T one, T two)
{
	if (one > two)
		return two;
	else
		return one;
}

template <class T>
T phy::maximum(T one, T two)
{
	if (one > two)
		return one;
	else
		return two;

}