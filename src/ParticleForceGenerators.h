#pragma once
#include "MathFunctions.h"
#include "ParticleForceGenerator.h"

namespace phy {
	class ParticleGravity : public ParticleForceGenerator
	{
	public:
		ParticleGravity(Vec2d gravity);

		//apply force of gravity to particle
		virtual void updateForce(Particle* particle, real timeDelta);

		//print out debug information
		virtual void printDebug();
		//get value of gravity
		Vec2d getGravity();

	private:
		//stores the force of gravity in Newtons
		Vec2d gravity;
	};
	class ParticleDrag : public ParticleForceGenerator
	{
	private:

	public:
		//coefficient of drag with respect to velocity
		real k1;
		//coefficient of drag with respect to velocity squared
		real k2;
		ParticleDrag(real velDragCoeff, real velSqDragCoeff);
		virtual void updateForce(Particle* particle, real timeDelta);

	};

	class ParticleSpring : public ParticleForceGenerator
	{
	public:
		// create new spring
		ParticleSpring(Particle* other, real springConstant, real restLength);
		//apply the spring force to THIS PARTICLE ONLY, requires two springs for both particles
		virtual void updateForce(Particle* particle, real timeDelta);
	private:
		//the particle at the other end of the spring
		Particle* other;
		//the spring constant in Nm-1
		real springConstant;
		//the rest length of the spring in Meters
		real restLength;
	};

	class ParticleAnchoredSpring : public ParticleForceGenerator
	{
	public:
		ParticleAnchoredSpring(Vec2d* anchor, real springConstant, real restLength);
		virtual void updateForce(Particle* particle, real timeDelta);
	private:
		// the anchor to which spring is attached
		Vec2d* anchor;
		//the spring constant in Nm-1
		real springConstant;
		//the rest length of the spring in Meters
		real restLength;
	};

	class ParticleElastic : public ParticleForceGenerator
	{
	public:
		ParticleElastic(Particle* anchor, real springConstant, real restLength);
		virtual void updateForce(Particle* particle, real timeDelta);
	private:
		// the anchor to which spring is attached
		Particle* other;
		//the spring constant in Nm-1
		real springConstant;
		//the rest length of the spring in Meters
		real restLength;
	};

	class ParticleAnchoredElastic : public ParticleForceGenerator
	{
	public:
		ParticleAnchoredElastic(Vec2d* anchor, real springConstant, real restLength);
		virtual void updateForce(Particle* particle, real timeDelta);
	private:
		// the anchor to which spring is attached
		Vec2d* anchor;
		//the spring constant in Nm-1
		real springConstant;
		//the rest length of the spring in Meters
		real restLength;
	};
};