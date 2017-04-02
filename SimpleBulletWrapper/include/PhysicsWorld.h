#ifndef PHYSICS_WORLD
#define PHYSICS_WORLD

//Physics
#include <btBulletDynamicsCommon.h>
#include "PhysicsCollision.h"
#include "glm/glm.hpp"

#include <vector>
#include <unordered_set>

class PhysicsObject;
class GravityObject;

typedef void(*PWCollisionCallback) (PhysicsCollision collision);

class PhysicsWorld
{
public:
	//Constructors, destructor
	PhysicsWorld(glm::vec3 gravity, bool useRadians);
	//Constructor taking multiple of G, acting in -Y direction
	PhysicsWorld(float gravityStrength, bool useRadians);
	PhysicsWorld(const PhysicsWorld &other);
	~PhysicsWorld();

	//Access functions
	glm::vec3 getGravity() const;
	bool getUsingRadians() const;

	//Functions to change the world
	void stepWorld(float deltaTime);
	void addPhysicsObject(PhysicsObject* object);
	void addGravityObject(GravityObject* object);
	void removePhysicsObject(PhysicsObject* object);

	//Collision checking functions
	bool areColliding(int ID1, int ID2);

	//Functions to register collision response functions
	void setCollisionFunction(PWCollisionCallback cf);
	void setCollisionStartFunction(PWCollisionCallback csf);
	void setCollisionEndFunction(PWCollisionCallback cef);


private:
	btBroadphaseInterface* broadphase;
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsWorld;

	bool usingRadians;

	std::vector<btRigidBody*> rigidBodies;
	std::vector<GravityObject*> gravityObjects;

	//Set of collisions that occurred in the previous tick
	std::unordered_set<PhysicsCollision, CollisionHasher> recordedCollisions;

	//Callback functions
	PWCollisionCallback collisionCallbackFunction = NULL;
	PWCollisionCallback collisionStartFunction = NULL;
	PWCollisionCallback collisionEndFunction = NULL;
};

#endif
