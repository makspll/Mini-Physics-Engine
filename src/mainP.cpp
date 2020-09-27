#pragma once
#include <iostream>
#include "MathFunctions.h"
#include "ParticlePhysicsWorld.h"
#include <chrono>

using namespace phy;


int mainP(int argc, char *argv[]) {

	ParticlePhysicsWorld* earth = new ParticlePhysicsWorld();

	Particle* particle1 = new Particle(1.0f, 5.0f, 0.1f);
	earth->addParticle(particle1);
	Vec2d* anchor = new Vec2d{ 0.0f,0.0f };

	ParticleForceGenerator* drag = new ParticleDrag(0.9f, 0.9f);
	ParticleForceGenerator* grav = new ParticleGravity(Vec2d{0.0f, -9.8f});
	ParticleForceGenerator* spring = new ParticleAnchoredSpring(anchor,1.0f,5.0f);

	earth->particleForces.add(particle1, drag);
	earth->particleForces.add(particle1, grav);
	earth->particleForces.add(particle1, spring);

	auto curr_frame_tick = std::chrono::high_resolution_clock::now();
	auto prev_frame_tick = curr_frame_tick;
	while (true)
	{
		prev_frame_tick = curr_frame_tick;
		curr_frame_tick = std::chrono::high_resolution_clock::now();
		std::chrono::duration<real> duration = curr_frame_tick - prev_frame_tick;
		earth->stepSimulation(duration.count());
		earth->debugParticles();
	}
	return 0;
}