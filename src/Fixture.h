#pragma once
#include "MathFunctions.h"
#include "IShape.h"
#include <vector>
#include <iomanip>

namespace phy {

	struct MassData
	{
		real inverseMass = 1;

		real inverseInertia = 1;

		Vec2d centroid = { 0.0f,0.0f };
	};

/*
Describes the physical 2d represenration of a rigid body
*/
	class Fixture
	{
	public:
		/*
		constructs a new fixture from shape and density
		*/
		Fixture(IShape * shape, real density);

		/*
		destructs the fixture and shape
		*/
		~Fixture();

		/*
		sets a new shape and density and recalculates
		*/
		void set(IShape * newShape, real density);


		// - - - getters and setters

		/*
		gets the centre of mass of fixture
		*/
		Vec2d getCentroid();

		/*
		TEMPORARY
		*/
		std::vector<Vec2d> getVerts(const Mat3x3 & transform) const;

		/*
		returns inverse mass of fixture
		*/
		inline real getInvMass() const 
		{
			return massInfo.inverseMass;
		}

		/*
		returns inverse moment of inertia of fixture
		*/
		inline real getInvInertia() const
		{
			return massInfo.inverseInertia;
		}

		/*
		returns the mass of fixture
		*/
		inline real getMass() const
		{
			return 1 / massInfo.inverseMass;
		}

		/*
		returns the moment of inertia of fixture
		*/
		inline real getInertia() const 
		{
			return 1 / massInfo.inverseInertia;
		}

	private:
		friend class Polygon;
		friend std::ostream& operator<<(std::ostream& os, const Fixture& fix);
		/*
		the shape of the body described
		*/
		IShape *shape;

		/*
		the density of the body
		*/
		real density = 1;

		/*
		holds the struct with mass and moment of inertia
		*/
		MassData massInfo = MassData();

	};


}