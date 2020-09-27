#include "Particle.h"
#include <assert.h>

using namespace phy;

Particle::Particle(real inX, real inY, real inMass)
	: position{ inX,inY }, velocity{ 0.0f,0.0f }, inverseMass(1.0f / inMass),
	acceleration{0.0f,0.0f}, forceAccum{ 0.0f,0.0f } {}

void Particle::integrate(real timeDelta)
{
	if (inverseMass <= 0.0f) return;

	assert(timeDelta > 0.0f);
	
	//update linear position
	position += (velocity * timeDelta);
	//resolve forces from the current time frame as an acceleration
	Vec2d resultingAcc = acceleration + (forceAccum * inverseMass);

	//update linear velocity with acceleration
	velocity +=  (resultingAcc * timeDelta);

	clearForceAccum();
}

void Particle::addForce(const Vec2d& force) 
{
	forceAccum += force;
}
void Particle::clearForceAccum()
{
	forceAccum.zeros();
}

void Particle::setInfiniteMass()
{
	inverseMass = 0.0f;
}
bool Particle::hasFiniteMass()
{
	return inverseMass;
}
void Particle::setMass(real mass) {
	inverseMass = 1.0f / mass;
}
real Particle::getMass() {
	return 1.0f / inverseMass;
}
real Particle::getInvMass() 
{
	return inverseMass;
}
void Particle::setPos(real inX, real inY) {
	position = {inX,inY};
}
void Particle::setPos(Vec2d vector)
{
	velocity = vector;
}
Vec2d Particle::getPos() {
	return position;
}
void Particle::setVel(real inX, real inY) {
	velocity = {inX, inY};
}
void Particle::setVel(Vec2d vector)
{
	velocity = vector;
}
Vec2d Particle::getVel() {
	return velocity;
}
void Particle::setAcc(real inX, real inY) {
	acceleration = {inX, inY};
}
Vec2d Particle::getAcc() {
	return velocity;
}


void Particle::printDebug()
{
	#ifdef _DEBUG
	std::cout << "Particle Log {"
		<< "Pos: [" << position[0] << ',' << position[1] << "] ,"
		<< " Vel: [" << velocity[0] << ',' << velocity[1] << "] ,"
		<< " Acc: [" << acceleration[0] << ',' << acceleration[1] << "] "
		<< " invMass: " << inverseMass << " } \n";	
	#else
	std::cout << "not implemented particle log file";
	#endif
}

