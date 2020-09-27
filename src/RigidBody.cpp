#pragma once
#include "RigidBody.h"
#include "Utility.h"
#include <allegro5/allegro_primitives.h>    

using namespace phy;


RigidBody::RigidBody()
{
}
RigidBody::~RigidBody()
{
	for (int i = 0; i < fixtures.size(); i++)
	{
		delete fixtures[i];
		fixtures.erase(fixtures.begin() + i);
	}
}
void RigidBody::calculateTransform()
{
	transform.setDirAndPos(ang,pos);
}

void RigidBody::calculateDerivedData()
{
	calculateTransform();
}


void RigidBody::clearForceAccumulators()
{
	forceAccum.zeros();
	torqueAccum = 0.0f;
}


void RigidBody::integrate(real timeDelta)
{
	if (!hasFiniteMass()) return;
	if (!isAwake) {
		assert(vel.squareMag() == 0);
		return;
	}
	assert(timeDelta >= 0.0f);

	//keep in mind acceleration  from forces is kept separate from 
	//inherent acceleration on object
	//we don't add acceleration and then clear it, 
	//we just clear forces instead 
	acc += forceAccum * inverseMass;
	angAcc += torqueAccum * inverseInertia;

	// linear
	pos += vel * timeDelta;


	vel += (acc) * timeDelta;
	

	// angular
	ang += angVel * timeDelta;
	//normalise to range (pi,-pi]
	ang = clampAngRad(ang);

	angVel += (angAcc) * timeDelta;


	// impose drag (temporary)
	vel = vel * powf(linearDamping,timeDelta);
	angVel = angVel * powf(angularDamping,timeDelta);



	//clear all forces
	clearForceAccumulators();

	//housekeeping
	calculateDerivedData();
}

void RigidBody::attachFixture(Fixture * fixture)
{
	fixtures.push_back(fixture);
	recalculateMassData();
}

void RigidBody::createFixture(IShape * shape, real density)
{
	Fixture * newFixture = new Fixture(shape, density);
	fixtures.push_back(newFixture);
	recalculateMassData();
}

void RigidBody::removeFixture(Fixture * fixture)
{
	for (int i = 0; i < fixtures.size(); i++)
	{
		if (fixture == fixtures[i])
		{
			delete fixtures[i];
			fixtures.erase(fixtures.begin() + i);
			return;
		}
	}
}

void RigidBody::recalculateMassData()
{
	//the centre of mass is just the average of all centres of mass 
	Vec2d averageCoM = {0.0f, 0.0f};

	//reset the mass values
	inverseMass = 0.0f;
	inverseInertia = 0.0f;

	// calculate mass first
	for (int i = 0; i < fixtures.size(); i++)
	{
		inverseMass += fixtures[i]->getInvMass();

		averageCoM += fixtures[i]->getCentroid();
	}
	centreOfMass = averageCoM /fixtures.size();

	for (int i = 0; i < fixtures.size();i++)
	{
		std::cout << fixtures[i]->getInertia();

		//the inertia needs to apply parallel axis theorem too
		inverseInertia += fixtures[i]->getInertia() + 
						  (fixtures[i]->getMass() * 
							(fixtures[i]->getCentroid() - centreOfMass).squareMag());

	}

	inverseInertia = 1.0f/inverseInertia;
}

void RigidBody::addForce(const Vec2d & worldForce)
{
	//force is through center of mass so no rotation
	forceAccum += worldForce;
	isAwake = true;
}

void RigidBody::addWorldForceAtWorldPoint(
	const Vec2d &worldForce, 
	const Vec2d &worldPoint)
{

	//with local body point just use the other function
	addWorldForceAtLocalPoint(transform.getInverse().transform(worldPoint), transform.transformDirInverse(worldForce));

	// also add a torque
	
	isAwake = true;
}

void RigidBody::addWorldForceAtLocalPoint(
	const Vec2d &worldForce, 
	const Vec2d &localPoint)
{
	//https://gamedev.net/forums/topic/592775-3d-rigid-body-how-to-apply-force-at-offset/4758043/

	//torque in 2d is the 3d vector pointing out 
	//so a 2d cross product gives it's magnitude

	torqueAccum += localPoint.Cross(worldForce); 

	//now this feels wrong, 
	forceAccum +=  worldForce;
	isAwake = true;
}

Vec2d RigidBody::pointToWorld(const Vec2d & localPoint) const
{
	return transform.transform(localPoint);
}
Vec2d RigidBody::pointToLocal(const Vec2d & worldPoint) const
{
	return transform.getInverse().transform(worldPoint);
}

bool RigidBody::hasFiniteMass() const
{
	return (inverseMass != 0.0f);
}

real RigidBody::getInvMass() const
{
	return inverseMass;
}

void RigidBody::setInfiniteMass()
{
	inverseMass = 0;
}

void RigidBody::setPos(const Vec2d & vector)
{
	pos = vector;
}

void RigidBody::setPos(const real inX, const real inY)
{
	pos = { inX,inY };
}

Vec2d RigidBody::getPos() const
{
	return pos;
}

void RigidBody::setVel(const Vec2d & vector)
{
	vel = vector;
}

void RigidBody::setVel(const real inX, const real inY)
{
	vel = { inX,inY };
}

Vec2d RigidBody::getVel() const
{
	return vel;
}

void RigidBody::setAcc(const real inX, const real inY)
{
	acc = { inX,inY };
}

Vec2d RigidBody::getAcc() const
{
	return acc;
}
void RigidBody::setMass(const real inMass)
{
	inverseMass = 1 / inMass;
}

real RigidBody::getMass() const
{
	return 1/inverseMass;
}

void RigidBody::setAng(const real inRadians)
{
	ang += inRadians;
}

void RigidBody::setAng(const Vec2d & inDir)
{
	ang += atan2(inDir[1],inDir[0]);
}

real RigidBody::getAng() const
{
	return ang;
}

void RigidBody::draw(){
	// draw each fixture
	for(int i =0; i < fixtures.size();i++){
		// get the positions of each vertex 
		std::vector<Vec2d> verts = fixtures[i]->getVerts(transform);
		
		for(int j =0; j< verts.size();j++){
			real x1 = verts[j].x();
			real y1 = verts[j].y();
			real x2 = verts[(j + 1) % verts.size()].x();
			real y2 = verts[(j + 1) % verts.size()].y();
			al_draw_line(x1,y1,x2,y2,al_map_rgb(0,255,0),2.0f);
		}
	}
};

void RigidBody::printDebug()
{
	#ifdef _DEBUG
	std::cout << "RigidBody Log {"
		<< "Pos: [" << std::setprecision(2) << pos[0]
		<< ',' << std::setprecision(2) << pos[1] << "] ,"
		<< " Vel: [" << std::setprecision(2) << vel[0] 
		<< ',' << std::setprecision(2) << vel[1] << "] ,"
		<< " Ang: [" << ang * (180/3.14)<< "] "
		<< " invMass: " << std::setprecision(2) << inverseMass << ","
		<< " } \n";
	#else
		std::cout << "not implemented log file";
	#endif
}

std::ostream& phy::operator<< (std::ostream& os, const RigidBody& body)
{
	os << "fixtures:";
	std::vector<Vec2d> vecs;
	vecs = body.fixtures[0]->getVerts(body.transform);

	for (int i = 0; i < vecs.size(); i++)
	{
		os << "[";
		os << i << vecs[i];
		os << "]";

	}
	os << "\n";
	return os;
}