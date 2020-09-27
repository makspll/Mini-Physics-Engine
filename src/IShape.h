#pragma once
#include "MathFunctions.h"
#include "AABB.h"
#include <iostream>

namespace phy {

	enum shapeType
	{
		polygon = 0,
		circle = 1
	};

	struct MassData;
	/*
	a shape represents a physical region in 2d space
	*/
	class IShape
	{
	public:
		/*
		destruct the shape
		*/
		virtual ~IShape(){};

		/*
		generate a tight fitting AABB bounding volume representation
		of this shape given its transform and write it to destAABB
		*/
		virtual void generateAABB(
			AABB		 &destAABB,
			const Mat3x3 &transform) = 0;

		/*
		calculate the mass of this shape based on its area and 
		density and write it to destMassData
		*/
		virtual void calculateMassData(
			MassData	&destMassData,
			real		density) = 0;

		virtual void print(std::ostream& where) const = 0;

		virtual std::vector<Vec2d> getWorldVertices(const Mat3x3 & transform) = 0;

		/*
		perform an intersection test with a ray 
		*/
			//////

		/*
		perform a point inclusion check
		*/
			//////

	};

	inline std::ostream& operator<< (std::ostream& os, IShape & shape)
	{
		shape.print(os);
		return os;
	}

}



