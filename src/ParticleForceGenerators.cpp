#pragma once
#include "ParticleForceGenerators.h"
#include "MathFunctions.h"

using namespace phy;

//ElasticAnchored
ParticleAnchoredElastic::ParticleAnchoredElastic(Vec2d* inAnchor, real inSpringConstant, real inRestLength)
	: anchor(inAnchor), springConstant(inSpringConstant), restLength(inRestLength) {}

void ParticleAnchoredElastic::updateForce(Particle* particle, real timeDelta)
{
	if (!particle->hasFiniteMass()) return;

	Vec2d vecToOther = *anchor - particle->getPos();
	real lenDelta = vecToOther.mag() - restLength;

	//check if compressing
	if (lenDelta < 0) return;

	real magnitude = springConstant * lenDelta;
	particle->addForce(magnitude * vecToOther.norm());
}

//SpringAnchored
ParticleAnchoredSpring::ParticleAnchoredSpring(Vec2d* inAnchor, real inSpringConstant, real inRestLength)
	: anchor(inAnchor), springConstant(inSpringConstant), restLength(inRestLength) {}

void ParticleAnchoredSpring::updateForce(Particle* particle, real timeDelta)
{
	if (!particle->hasFiniteMass()) return;

	Vec2d vecToOther = *anchor - particle->getPos();
	real magnitude = springConstant * (vecToOther.mag() - restLength);
	particle->addForce(magnitude * vecToOther.norm());
}

//Drag
ParticleDrag::ParticleDrag(real velCoeff, real velSqCoeff)
	:k1(velCoeff), k2(velSqCoeff) {}

void ParticleDrag::updateForce(Particle* particle, real timeDelta)
{
	if (!particle->hasFiniteMass()) return;

	Vec2d force = particle->getVel();
	real dragCoeff = force.mag();
	dragCoeff = (k1 * dragCoeff) + (k2 * dragCoeff * dragCoeff);

	force = force.norm() * -dragCoeff;
	particle->addForce(force);
}

//Elastic
ParticleElastic::ParticleElastic(Particle* otherParticle, real inSpringConstant, real inRestLength)
	: other(otherParticle), springConstant(inSpringConstant), restLength(inRestLength) {}

void ParticleElastic::updateForce(Particle* particle, real timeDelta)
{
	if (!particle->hasFiniteMass()) return;


	Vec2d vecToOther = other->getPos() - particle->getPos();
	real lenDelta = vecToOther.mag() - restLength;

	//check if compressing
	if (lenDelta < 0) return;

	real magnitude = springConstant * lenDelta;
	particle->addForce(magnitude * vecToOther.norm());
}

//Gravity
ParticleGravity::ParticleGravity(Vec2d inGravity)
	: gravity(inGravity) {}

void ParticleGravity::updateForce(Particle* particle, real timeDelta) {
	if (!particle->hasFiniteMass()) return;
	particle->addForce(gravity * particle->getMass());
}
void ParticleGravity::printDebug()
{
#ifdef _DEBUG
	std::cout << "ForceGenerator pGravity Log {"
		<< "Gravity: [" << gravity[0] << ',' << gravity[1] << "] }";
#else
	std::cout << "not implemented log file";
#endif
}

Vec2d ParticleGravity::getGravity()
{
	return gravity;
}

//Spring
ParticleSpring::ParticleSpring(Particle* otherParticle, real inSpringConstant, real inRestLength)
	: other(otherParticle), springConstant(inSpringConstant), restLength(inRestLength) {}

void ParticleSpring::updateForce(Particle* particle, real timeDelta)
{
	if (!particle->hasFiniteMass()) return;
	Vec2d vecToOther = other->getPos() - particle->getPos();
	real magnitude = springConstant * (vecToOther.mag() - restLength);
	particle->addForce(magnitude * vecToOther.norm());
}