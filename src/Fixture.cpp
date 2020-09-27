#pragma once
#include "Fixture.h"
#include "IShape.h"

using namespace phy;

Fixture::Fixture(IShape * inshape, real indensity)

{
	shape = inshape;
	shape->calculateMassData(massInfo, density);
}


Fixture::~Fixture()
{
	delete &massInfo;
	delete shape;
}


void Fixture::set(IShape * newShape, real density)
{
	//delete old shape
	delete &shape;
	//change the pointer
	delete shape;
	shape = newShape;

	//recalculate everything
	shape->calculateMassData(massInfo, density);
}

Vec2d phy::Fixture::getCentroid()
{
	return massInfo.centroid;
}

std::vector<Vec2d> phy::Fixture::getVerts(const Mat3x3 & transform) const
{
	return shape->getWorldVertices(transform);
}

std::ostream& phy::operator<<(std::ostream& os, const Fixture& fix)
{
	os << '\n' << "Mass:" << std::setprecision(3) <<
		fix.getMass() <<
		" Inertia: " << std::setprecision(3) <<
		fix.getInertia() <<
		*fix.shape;

	return os;
}