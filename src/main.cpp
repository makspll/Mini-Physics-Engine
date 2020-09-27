#pragma once
#include <iostream>
#include "RigidBody.h"
#include <chrono>
#include "PhysicsWorld.h"
#include "IForceGenerator.h"
#include "RigidBodyFixedSpring.h"
#include "RigidBodySimpleGravity.h"
#include "RigidBodySimpleGravity.h"
#include "Fixture.h"
#include "Polygon.h"

using namespace phy;

int main(int argc, char *argv[]) {

	PhysicsWorld* world = new PhysicsWorld();

	IForceGenerator * grav = new RigidBodyFixedSpring(Vec2d(1.0f,0.0f),Vec2d(0.0f,1.0f),500.0f,1.0f);
	IForceGenerator * grav2 = new RigidBodyFixedSpring(Vec2d(1.0f, 0.0f), Vec2d(0.0f, 1.0f), 500.0f, 1.0f);
	IForceGenerator * gravity = new RigidBodySimpleGravity(Vec2d(0.0f,-9.8f));
	RigidBody* body = new RigidBody();

	Polygon a = Polygon();
	Vec2d asd[] = { Vec2d(1.0f,1.0f),Vec2d(-1.0f,1.0f),Vec2d(-1.0f,-1.0f),Vec2d(1.0f,-1.0f) };
	a.setShape(asd,4);

	Fixture abs = Fixture(&a,1);

	
	std::cout << abs;
	body->createFixture(&abs);
	world->addObject(body);
	world->forces.addForceGenerator(body, gravity);
	world->forces.addForceGenerator(body, grav2);
	/*
	for (int i = 0; i < 100; i++)
	{
		RigidBody* body = new RigidBody();
		body->setMass(100.0f);
		body->setPos(i*1.0f, 0.0f);
		world->addObject(body);
		//world->forces.addForceGenerator(body, grav);
		//world->forces.addForceGenerator(body, grav2);
		world->forces.addForceGenerator(body, gravity);
	}
	*/

	world->setUpSimulation();

	auto curr_frame_tick = std::chrono::high_resolution_clock::now();
	auto prev_frame_tick = curr_frame_tick;
	while (true)
	{
		prev_frame_tick = curr_frame_tick;
		curr_frame_tick = std::chrono::high_resolution_clock::now();
		std::chrono::duration<real> duration = curr_frame_tick - prev_frame_tick;

		world->stepSimulation(std::chrono::duration_cast<std::chrono::duration<double>>(duration).count());
		std::cout << *body;

		//std::cout << std::chrono::duration_cast<std::chrono::duration<double>>(duration).count() << "\n";
		//world->printDebug();
	}
	return 0;
}

