#pragma once
#include "ParticlePhysicsWorld.h"

using namespace phy;

void ParticlePhysicsWorld::stepSimulation(real deltaTime) {
	for (int i = 0; i < particles.size(); i++)
	{
		particleForces.updateForces(deltaTime);
		particles[i]->integrate(deltaTime);
	};
};

void ParticlePhysicsWorld::addParticle(Particle* inParticle)
{
	particles.push_back(inParticle);
}

void ParticlePhysicsWorld::removeParticle(Particle* inParticle)
{
	for (int i = 0; i < particles.size(); i++)
	{
		if (inParticle == particles[i])
		{
			delete particles[i];
			particles.erase(particles.begin() + i);
			return;
		}
	}
}

void ParticlePhysicsWorld::clearParticles()
{
	for (int i = 0; i < particles.size(); i++)
	{
		delete particles[i];
		particles.erase(particles.begin() + i);
	}
}

ParticlePhysicsWorld::~ParticlePhysicsWorld()
{
	clearParticles();
}

void ParticlePhysicsWorld::debugParticles()
{
	for (int i = 0; i < particles.size(); i++)
	{
		particles[i]->printDebug();
	}
}