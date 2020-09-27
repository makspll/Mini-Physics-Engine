#pragma once
#include "AABB.h"

using namespace phy;

AABB::AABB(const Vec2d &min, const Vec2d &max)
	:min(min), max(max) {}

AABB::AABB(const AABB &a1, const AABB &a2)
{
	//iterate through axis and for both x and y find which AABB's
	//min and max is the min and max for the combined AABB

	for (int i = 0; i < 2; i++)
	{
		min[i] = std::min(a1.min[i], a2.min[i]);
		max[i] = std::max(a1.max[i], a2.max[i]);
	}
}


bool AABB::overlaps(const AABB &o)
{
	//exit without intersection if separated along any axis
	if (max[0] < o.min[0] || min[0] > o.max[0]) return 0;
	if (max[1] < o.min[1] || min[1] > o.max[1]) return 0;

	//overlapping on all axes means intersection
	return 1;
}

void AABB::set(const Vec2d & minI, const Vec2d & maxI)
{
	min = minI;
	max = maxI;
}