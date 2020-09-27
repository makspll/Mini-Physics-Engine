#include "ParticleForceRegistry.h"


using namespace phy;

void ParticleForceRegistry::add(Particle* particle, ParticleForceGenerator* fg)
{
	registrations.push_back(ParticleForceRegistration(particle, fg));
}

void ParticleForceRegistry::remove(Particle* particle, ParticleForceGenerator* fg)
{
	for (int i = 0; i < registrations.size(); i++)
	{
		if (particle == registrations[i].particle && fg == registrations[i].fg)
		{
			registrations.erase(registrations.begin() + i);
			return;
		}
	}
}

void ParticleForceRegistry::clear()
{
	registrations.clear();
}

void ParticleForceRegistry::updateForces(real timeDelta)
{
	for (int i = 0; i < registrations.size(); i++)
	{
		registrations[i].fg->updateForce(registrations[i].particle,timeDelta);
	}
}

void ParticleForceRegistry::ParticleForceRegistration::printDebug()
{
	#ifdef _DEBUG
	std::cout << "pForceRegistration log { "
		<< "Particle&: " << particle << ", ";
	fg->printDebug();
	std::cout << " }";
	#else
	#endif
}

void ParticleForceRegistry::printDebug()
{
	#ifdef _DEBUG
	std::cout << "pForceRegistry log { \n ";
	for (int i = 0; i < registrations.size(); i++)
	{
		registrations[i].printDebug();
		std::cout << '\n';
	}
	std::cout << " } \n";
	#else
	std::cout << "not implemented log file";
	#endif
}

phy::ParticleForceRegistry::ParticleForceRegistry()
{
}


ParticleForceRegistry::~ParticleForceRegistry()
{
	clear();
}

ParticleForceRegistry::ParticleForceRegistration::ParticleForceRegistration(Particle* inParticle, ParticleForceGenerator* inFg)
	: particle(inParticle), fg(inFg) {}

