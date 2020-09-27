#pragma once
#include "MathFunctions.h"
#include "IBoundingBolume.h"

namespace phy {
	struct AABB : IBoundingVolume
	{
		/*
		create a new AABB from min max vectors
		*/
		AABB(const Vec2d &min,const Vec2d &max);

		/*
		create a new AABB from two other AABB's
		*/
		AABB(const AABB &a1, const AABB &a2);

		
		/*
		test weather this AABB overlaps with another
		*/
		bool overlaps(const AABB &other);

		/*
		sets the min and max 
		*/
		void set(const Vec2d & minI,const Vec2d & maxI);

		/*
		transforms the AABB to fit a new position given by
		a transformation to the new position
		*/
		//void update();

	protected:
//		the region AABB = 
//		{(x,y,z) | min.x <= x <= max.x , min.y <= max.y}

		/*
		the lower left corner (world)
		*/
		Vec2d min;
		/*
		the upper right corner (world)
		*/
		Vec2d max;
	};
}