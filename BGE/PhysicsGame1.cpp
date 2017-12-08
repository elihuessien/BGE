#include "PhysicsGame1.h"
#include "PhysicsController.h"
#include "Sphere.h"
#include "PhysicsCamera.h"
#include "Box.h"
#include "Cylinder.h"
#include "Steerable3DController.h"
#include "Ground.h"
#include "Content.h"
#include <btBulletDynamicsCommon.h>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
#include "VectorDrawer.h"
#include "Utils.h"

using namespace BGE;

//TODO floats x, y, z and speed;
float x, y, z, speed, rotation = 90;
float maxRotation = 100.f;
float minRotation = 80.f;
int height = 3;
int width = 7 ;

PhysicsGame1::PhysicsGame1(void)
{
}

PhysicsGame1::~PhysicsGame1(void)
{
}

std::shared_ptr<GameComponent> station;

bool PhysicsGame1::Initialise() 
{	
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();

	//setting the world variables
	btVector3 worldMin(-1000, -1000, -1000);
	btVector3 worldMax(1000, 1000, 1000);
	dynamicsWorld->setGravity(btVector3(0, -5, 0));
	glm::vec3 Black(0, 0, 0);
	glm::vec3 White(2.5f, 1.f, 0.5f);
	glm::vec3 Red(2.5f, 0.5f, 0);
	glm::vec3 Red2(2.5f, 0, 0);
	glm::vec3 Yellow(2.5f, 2.5f, 0);
	x = 0; y = 6; z = -3; speed = 500;


	

	//create the head
	btTransform t1, t2;
	shared_ptr<PhysicsController> head = physicsFactory->CreateBox(1, 1, 6, glm::vec3(x, y, z+3), glm::quat());
	head->transform->diffuse = Red;

	shared_ptr<PhysicsController> wheel = physicsFactory->CreateSphere(0.5, glm::vec3(x, y, z+7), glm::angleAxis(90.f, glm::vec3(0, 0, 1)));
	wheel->transform->diffuse = White;
	btHingeConstraint * frontwheel = new btHingeConstraint(*head->rigidBody, *wheel->rigidBody, btVector3(0, 0, 4), btVector3(0, 0, 0), btVector3(0, 0, 0), btVector3(0, 0.001, 0), true);
	frontwheel->setLimit(0, 0.001);
	dynamicsWorld->addConstraint(frontwheel);

	/*Other idea for dynamic steering wheel
	shared_ptr<PhysicsController> wheel = physicsFactory->CreateCylinder(0.5, 2, glm::vec3(0, 10, 7), glm::angleAxis(rotation, glm::vec3(0, 0, 1)));
	btHingeConstraint *steer = new btHingeConstraint(*head->rigidBody, *wheel->rigidBody, btVector3(0, 0, 3), btVector3(0, 0, -1), btVector3(0, 0, 0), btVector3(0, 0, 1), false);

	dynamicsWorld->addConstraint(steer);
	*/

	//Part of the original ideas
	//shared_ptr<PhysicsController> headnib = physicsFactory->CreateSphere(0.5, glm::vec3(0, 10, 7), glm::quat());
	/*
	t1.setIdentity();
	t2.setIdentity();
	t1.setOrigin(btVector3(0, 0, 3));
	t2.setOrigin(btVector3(0, 0, -1));
	btFixedConstraint *nib1 = new btFixedConstraint(*head->rigidBody, *headnib->rigidBody, t1, t2);
	dynamicsWorld->addConstraint(nib1);
	*/

	//create handles
	shared_ptr<PhysicsController> handle1 = physicsFactory->CreateCylinder(0.3, 0.5, glm::vec3(x+1, y, z+1), glm::quat());
	shared_ptr<PhysicsController> handle2 = physicsFactory->CreateCylinder(0.3, 0.5, glm::vec3(x-1, y, z+1), glm::quat());
	handle1->transform->diffuse = Black;
	handle2->transform->diffuse = Black;
	t1.setIdentity();
	t2.setIdentity();
	t1.setOrigin(btVector3(1, 0, -2));
	t2.setOrigin(btVector3(0, 0, 0));
	t2.setRotation(GLToBtQuat(glm::angleAxis(90.f, glm::vec3(0, 0, 1))));
	btFixedConstraint *lefthandle = new btFixedConstraint(*head->rigidBody, *handle1->rigidBody, t1, t2);
	dynamicsWorld->addConstraint(lefthandle);
	t1.setIdentity();
	t2.setIdentity();
	t1.setOrigin(btVector3(-1, 0, -2));
	t2.setOrigin(btVector3(0, 0, 0));
	t2.setRotation(GLToBtQuat(glm::angleAxis(90.f, glm::vec3(0, 0, 1))));
	btFixedConstraint *righthandle = new btFixedConstraint(*head->rigidBody, *handle2->rigidBody, t1, t2);
	dynamicsWorld->addConstraint(righthandle);


	//create seat
	shared_ptr<PhysicsController> chair = physicsFactory->CreateBox(1, 1, 2, glm::vec3(x, y-1, z - 2), glm::quat());
	chair->transform->diffuse = Red2;
	t1.setIdentity();
	t2.setIdentity();
	t1.setOrigin(btVector3(0, -1, -5));
	t2.setOrigin(btVector3(0, 0, 0));
	t2.setRotation(GLToBtQuat(glm::angleAxis(30.f, glm::vec3(1, 0, 0))));
	btFixedConstraint *seat = new btFixedConstraint(*head->rigidBody, *chair->rigidBody, t1, t2);
	dynamicsWorld->addConstraint(seat);


	//create first support
	shared_ptr<PhysicsController> support1 = physicsFactory->CreateBox(1, 2, 1, glm::vec3(x+1, y-2, z-1), glm::quat());
	support1->transform->diffuse = Red;
	shared_ptr<PhysicsController> supnib1 = physicsFactory->CreateSphere(0.5, glm::vec3(x+1, y-4, z-1), glm::quat());
	supnib1->transform->diffuse = Yellow;
	t1.setIdentity();
	t2.setIdentity();
	t1.setOrigin(btVector3(0, -1, 0));
	t2.setOrigin(btVector3(0, 1, 0));
	btFixedConstraint *nib2 = new btFixedConstraint(*support1->rigidBody, *supnib1->rigidBody, t1, t2);
	//Wanted a rotating wheel to reduce drag
	//btHingeConstraint *nib2 = new btHingeConstraint(*support1->rigidBody, *supnib1->rigidBody, btVector3(0, -1, 0), btVector3(0, 1, 0), btVector3(0,0,0), btVector3(-1,0,0));
	dynamicsWorld->addConstraint(nib2);
	t1.setIdentity();
	t2.setIdentity();
	t1.setOrigin(btVector3(0, 0, -4));
	t2.setOrigin(btVector3(-1, 2, 0));
	btFixedConstraint *leg1 = new btFixedConstraint(*head->rigidBody, *support1->rigidBody, t1, t2);
	dynamicsWorld->addConstraint(leg1);

	//create second support
	shared_ptr<PhysicsController> support2 = physicsFactory->CreateBox(1, 2, 1, glm::vec3(x-1, y-2, z-1), glm::quat());
	support2->transform->diffuse = Red;
	shared_ptr<PhysicsController> supnib2 = physicsFactory->CreateSphere(0.5, glm::vec3(x-1, y-4, z-1), glm::quat());
	supnib2->transform->diffuse = Yellow;
	t1.setIdentity();
	t2.setIdentity();
	t1.setOrigin(btVector3(0, -1, 0));
	t2.setOrigin(btVector3(0, 1, 0));
	btFixedConstraint *nib3 = new btFixedConstraint(*support2->rigidBody, *supnib2->rigidBody, t1, t2);
	//Wanted a rotating wheel to reduce drag
	//btHingeConstraint *nib3 = new btHingeConstraint(*support2->rigidBody, *supnib2->rigidBody, btVector3(0, -1, 0), btVector3(0, 1, 0), btVector3(0,0,0), btVector3(-1,0,0));
	dynamicsWorld->addConstraint(nib3);
	t1.setIdentity();
	t2.setIdentity();
	t1.setOrigin(btVector3(0, 0, -4));
	t2.setOrigin(btVector3(1, 2, 0));
	btFixedConstraint *leg2 = new btFixedConstraint(*head->rigidBody, *support2->rigidBody, t1, t2);
	dynamicsWorld->addConstraint(leg2);

	//motor
	/*       Original method of propulsion
	shared_ptr<PhysicsController> motor = physicsFactory->CreateBox(2, 4, 1, glm::vec3(0, 7.5, -1), glm::quat());
	btHingeConstraint * leftjoint = new btHingeConstraint(*support1->rigidBody, *motor->rigidBody, btVector3(-2, -0.5, 0), btVector3(0, 0, 0), btVector3(0, 0, 0), btVector3(1, 0, 0), true);
	btHingeConstraint * rightjoint = new btHingeConstraint(*support2->rigidBody, *motor->rigidBody, btVector3(2, -0.5, 0), btVector3(0, 0, 0), btVector3(0, 0, 0), btVector3(1, 0, 0), true);
	leftjoint->setLimit(0, 0.1);
	rightjoint->setLimit(0, 0.1);
	leftjoint->enableAngularMotor(true, 600, 300);
	rightjoint->enableAngularMotor(true, 600, 300);
	dynamicsWorld->addConstraint(leftjoint);
	dynamicsWorld->addConstraint(rightjoint);
	*/
	shared_ptr<PhysicsController> motor = physicsFactory->CreateCylinder(1.5, 1, glm::vec3(x, y-3, z-1), glm::angleAxis(90.f, glm::vec3(0, 0, 1)));
	motor->transform->diffuse = White;
	btHingeConstraint * leftjoint = new btHingeConstraint(*supnib1->rigidBody, *motor->rigidBody, btVector3(-1, 1, 0), btVector3(0, 0, 0), btVector3(0, 0, 0), btVector3(0, -1, 0), true);
	btHingeConstraint * rightjoint = new btHingeConstraint(*supnib2->rigidBody, *motor->rigidBody, btVector3(1, 1, 0), btVector3(0, 0, 0), btVector3(0, 0, 0), btVector3(0, -1, 0), true);
	leftjoint->setLimit(0, 0.1);
	rightjoint->setLimit(0, 0.1);
	leftjoint->enableAngularMotor(true, speed, 300);
	rightjoint->enableAngularMotor(true, speed, 300);
	dynamicsWorld->addConstraint(leftjoint);
	dynamicsWorld->addConstraint(rightjoint);

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			shared_ptr<PhysicsController> tower = physicsFactory->CreateBox(1, 1, 1, glm::vec3(x +(j*1), 1 + (i * 1), z + 15), glm::quat());
		}
	}

	if (!Game::Initialise()) {
		return false;
	}

	camera->transform->position = glm::vec3(0,10, 20);
	return true;
}


void BGE::PhysicsGame1::Update(float timeDelta)
{
	//code for rotating the steering wheel

	if (keyState[SDL_SCANCODE_LEFT])
	{
		if (rotation > minRotation)
			rotation++;
	}
	if (keyState[SDL_SCANCODE_RIGHT])
	{
		if (rotation < maxRotation)
			rotation--;
	}
	Game::Update(timeDelta);
}

void BGE::PhysicsGame1::Cleanup()
{
	Game::Cleanup();
}