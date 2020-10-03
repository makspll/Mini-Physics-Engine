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
#include <allegro5/allegro.h>
#include <allegro5/allegro_primitives.h>    
#include "PhysicsStepper.h"

using namespace phy;

int main(int argc, char *argv[]) {


	// setup allegro display
    al_init();
	al_init_primitives_addon();  	

	ALLEGRO_DISPLAY * display = al_create_display(640, 480);

	std::cout << "Press enter to simulate \n";
    // wait for space bar
	std::cin.get();

	// set up the physics world
	PhysicsWorld* world = new PhysicsWorld();
	ParticlePhysicsWorld* pWorld = new ParticlePhysicsWorld();

	Vec2d worldCenter = Vec2d(640.0f/2.0,480.0f/2.0);

	IForceGenerator* spring = new RigidBodyFixedSpring(Vec2d(5,0),worldCenter,50.0f,10.0f);
	IForceGenerator* spring2 = new RigidBodyFixedSpring(Vec2d(-5,0),worldCenter,50.0f,10.0f);

	IForceGenerator * gravity = new RigidBodySimpleGravity(Vec2d(0.0f,9.8f));
	RigidBody* body = new RigidBody();
	body->setPos(worldCenter);
	Polygon a = Polygon();
	Vec2d polygonVerts[] = { Vec2d(10.0f,10.0f),Vec2d(-10.0f,10.0f),Vec2d(-10.0f,-10.0f),Vec2d(10.0f,-10.0f) };

	a.setShape(polygonVerts,4);

	Fixture abs = Fixture(&a,1);


	body->attachFixture(&abs);
	world->addObject(body);
	world->forces.addForceGenerator(body,gravity);
	world->forces.addForceGenerator(body,spring);
	world->forces.addForceGenerator(body,spring2);
	world->setUpSimulation();

	std::cout  <<  abs << body;

	PhysicsStepper* stepper = new PhysicsStepper(world,pWorld);

	while(true){
		body->draw();
		al_flip_display();  	
		al_clear_to_color(al_map_rgb(0, 0, 0));
		stepper->stepSimulation();
	}

	return 0;
}

