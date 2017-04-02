#include "include/GravityObject.h"
#include "include/PhysicsUtilities.h"
#include "include/PhysicsWorld.h"

GravitySphere::GravitySphere(float radius, float surfaceGravity, glm::vec3 position, PhysicsWorld* world)
{
	this->radius = radius;
	this->surfaceGravity = surfaceGravity * 10;
	this->centreOfMass = position;

	btCollisionShape* cShape = new btSphereShape(radius);

	btDefaultMotionState* mState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(position.x, position.y, position.z)));

	btRigidBody* rBody = PhysicsUtilities::constructRigidBody(cShape, mState, false, 0);

	//IMPORTANT: Store the physics data structures in the superclass fields!
	Init(cShape, mState, rBody, world);
	world->addGravityObject(this);
}

void GravitySphere::applyGravity(btRigidBody* obj)
{
	glm::vec3 objDisplacement(centreOfMass.x - obj->getCenterOfMassPosition().x(), centreOfMass.y - obj->getCenterOfMassPosition().y(), centreOfMass.z - obj->getCenterOfMassPosition().z());
	float objDistance = glm::length(objDisplacement);
	float objMass = 1.0f / obj->getInvMass();

	glm::vec3 force = objDisplacement * objMass * surfaceGravity * radius * radius / (objDistance * objDistance * objDistance);

	//Testing the surface gravity
	//std::cout << "Surface gravity acceleration: " << surfaceGravity * radius * radius / (objDistance * objDistance) << std::endl;

	obj->applyCentralForce(btVector3(force.x, force.y, force.z));
}