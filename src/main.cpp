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
	Vec2d worldCenter = Vec2d(640.0f/2.0,480.0f/2.0);

	IForceGenerator* spring = new RigidBodyFixedSpring(Vec2d(1,0),worldCenter,10000.0f,5.0f);

	IForceGenerator * gravity = new RigidBodySimpleGravity(Vec2d(0.0f,50.8f));
	RigidBody* body = new RigidBody();
	body->setPos(worldCenter);
	Polygon a = Polygon();
	Vec2d polygonVerts[] = { Vec2d(20.0f,20.0f),Vec2d(-20.0f,20.0f),Vec2d(-20.0f,-20.0f),Vec2d(20.0f,-20.0f) };

	a.setShape(polygonVerts,4);

	Fixture abs = Fixture(&a,1);


	std::cout << abs;
	body->createFixture(&abs);
	world->addObject(body);
	world->forces.addForceGenerator(body, gravity);
	world->forces.addForceGenerator(body,spring);
	world->setUpSimulation();


	// timing settings
	auto curr_frame_tick = std::chrono::high_resolution_clock::now();
	auto prev_frame_tick = curr_frame_tick;

	// run the physics/graphics loop

	while(true){
		prev_frame_tick = curr_frame_tick;
		curr_frame_tick = std::chrono::high_resolution_clock::now();
		std::chrono::duration<real> duration = curr_frame_tick - prev_frame_tick;

		body->draw();
		al_flip_display();  	
		al_clear_to_color(al_map_rgb(0, 0, 0));
		world->stepSimulation(std::chrono::duration_cast<std::chrono::duration<double>>(duration).count());

		//std::cout << std::chrono::duration_cast<std::chrono::duration<double>>(duration).count() << "\n";
		//world->printDebug();
	}

	return 0;
}

