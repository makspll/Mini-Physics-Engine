#pragma once
#include <vector>
#include "MathFunctions.h"
#include "Particle.h"
#include "ParticleForceRegistry.h"
#include "ParticleForceGenerators.h"

namespace phy {
	class ParticlePhysicsWorld 
	{
	public:
		//integrate all bodies, time step in seconds
		void stepSimulation(real deltaTime);
		//add a particle object to the registry
		void addParticle(Particle* particle);
		//remove a particle object from the registry
		void removeParticle(Particle* particle);
		//clear all particles from the registry
		void clearParticles();
		//print debug info for particles
		void debugParticles();
		//stores forcesGenerators
		ParticleForceRegistry particleForces;

		~ParticlePhysicsWorld();
	private:
		typedef std::vector<Particle*> Registry;
		//stores pointers to particles
		Registry particles;
	};

}