#pragma once
#include "MathFunctions.h"

namespace phy {
	class Particle final
	{
	public:
		Particle(real inX = 0.0f, real inY = 0.0f, real inMass = 1.0f);

		//make the particle static
		void setInfiniteMass();
		//check if the particle is static
		bool hasFiniteMass();

		//update position velocity and acceleration
		void integrate(real timeDelta);

		//apply a force on the particle (not instantaneous)
		void addForce(const Vec2d& force);

		//clear the force buffer (will be zero outside of the integrator)
		void clearForceAccum();

		//set the mass of the particle (not inverse mass)
		void setMass(real inMass);
		//get the mass of the particle (not inverse mass)
		real getMass();

		//get the inverse mass of the particle
		real getInvMass();

		//set the position of the particle
		void setPos(real inX, real inY);
		//get the position of the particle
		Vec2d getPos();
		//set position from vector
		void setPos(Vec2d vector);

		//set the velocity of the particle
		void setVel(real inX, real inY);
		//get the velocity of the particle
		Vec2d getVel();

		//setVelocity as vector
		void setVel(Vec2d vector);

		//set the acceleration of the particle (equivalent to applying a constant force as long as acceleration is non-zero)
		void setAcc(real inX, real inY);
		//get the acceleration of the particle (it will be non-zero only if set manually to be so)
		Vec2d getAcc();


		//print out information about the particle
		void printDebug();

	protected:
		//	stores the poosition of the particle
		Vec2d position;
		//	stores the velocity of the particle in ms-1
		Vec2d velocity;
		//	stores the acceleration of the particle in ms-2
		Vec2d acceleration;
		//	stores the sum of all forces acting in each frame on the particle in Newtons
		Vec2d forceAccum;

		// storing inverse mass in Kg so that objects of infinite mass can be represented
		real inverseMass;

	};
};