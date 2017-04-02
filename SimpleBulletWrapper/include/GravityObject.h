#ifndef GRAVITY_WRAPPER_OBJECT
#define GRAVITY_WRAPPER_OBJECT

#include "PhysicsObject.h"

class GravityObject :public PhysicsObject
{
public:
	virtual void applyGravity(btRigidBody* obj) = 0;
};

class GravitySphere :public GravityObject
{
public:
	GravitySphere(float radius, float surfaceGravity, glm::vec3 position, PhysicsWorld* world);
	void applyGravity(btRigidBody* obj);
private:
	float surfaceGravity;
	float radius;
	glm::vec3 centreOfMass;
};

#endif