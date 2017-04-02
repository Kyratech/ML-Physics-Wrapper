#include "include/PhysicsWorld.h"
#include "include/PhysicsObject.h"
#include "include/GravityObject.h"
#include <algorithm>
#include <iostream>
#include <stdint.h>

PhysicsWorld::PhysicsWorld(glm::vec3 gravity, bool useRadians)
{
	broadphase = new btDbvtBroadphase();

	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);

	solver = new btSequentialImpulseConstraintSolver;

	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

	dynamicsWorld->setGravity(btVector3(gravity.x, gravity.y, gravity.z));

	usingRadians = useRadians;
}

/*
 * Uses the assumption that gravity is going to work in the -Y direction, as other constructs treat +Y as 'up'
 * Instead of setting the gravity strength directly, uses a more accessible multiple of G
*/
PhysicsWorld::PhysicsWorld(float gravityStrength, bool useRadians)
{
	broadphase = new btDbvtBroadphase();

	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);

	solver = new btSequentialImpulseConstraintSolver;

	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

	//Round G to 10 m.s^(-2)
	dynamicsWorld->setGravity(btVector3(0, gravityStrength * -10, 0));

	usingRadians = useRadians;
}

PhysicsWorld::PhysicsWorld(const PhysicsWorld &other)
{
	broadphase = new btDbvtBroadphase();

	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);

	solver = new btSequentialImpulseConstraintSolver;

	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

	glm::vec3 otherGravity = other.PhysicsWorld::getGravity();
	dynamicsWorld->setGravity(btVector3(otherGravity.x, otherGravity.y, otherGravity.z));
}

PhysicsWorld::~PhysicsWorld()
{
	delete dynamicsWorld;
	delete solver;
	delete collisionConfiguration;
	delete dispatcher;
	delete broadphase;
}

glm::vec3 PhysicsWorld::getGravity() const
{
	btVector3 gravity = dynamicsWorld->getGravity();
	return glm::vec3(gravity.x(), gravity.y(), gravity.z());
}

void PhysicsWorld::stepWorld(float deltaTime)
{
    for(int gravityObject = 0; gravityObject < gravityObjects.size(); gravityObject++)
    {
        for(int rigidBody = 0; rigidBody < rigidBodies.size(); rigidBody++)
        {
            gravityObjects[gravityObject]->applyGravity(rigidBodies[rigidBody]);
        }
    }

    /*
	for each (GravityObject* gravitySource in gravityObjects)
	{
		for each (btRigidBody* obj in rigidBodies)
		{
			gravitySource->applyGravity(obj);
		}
	}
	*/

	//Max sub-steps of 10 allows simulation to run as slow as 6fps before losing time
	//Could possibly make this dynamic, but fixed value is sufficient for casual use
	dynamicsWorld->stepSimulation(deltaTime, 10);

	/* Collision detection */

	//Store the new collisions
	std::unordered_set<PhysicsCollision, CollisionHasher> newCollisions;

	int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++)
	{
		//Find the objects involved
		btPersistentManifold* contact = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject* coA = contact->getBody0();
		const btCollisionObject* coB = contact->getBody1();

		//See if any of the contact points for the pair is less than 0
		//Indicates actual contact
		int numContacts = contact->getNumContacts();
		for (int j = 0; j < numContacts; j++)
		{
			btManifoldPoint& point = contact->getContactPoint(j);
			if (point.getDistance() < 0.0f)
			{
				//Get the relevant information
				intptr_t ID1 = (intptr_t) coA->getUserPointer();
				intptr_t ID2 = (intptr_t) coB->getUserPointer();
				glm::vec3 posA(point.getPositionWorldOnA().x(), point.getPositionWorldOnA().y(), point.getPositionWorldOnA().z());
				glm::vec3 posB(point.getPositionWorldOnB().x(), point.getPositionWorldOnB().y(), point.getPositionWorldOnB().z());
				glm::vec3 normal(point.m_normalWorldOnB.x(), point.m_normalWorldOnB.y(), point.m_normalWorldOnB.z());

				PhysicsCollision collision(ID1, ID2, posA, posB, normal);

				//If both participants in the collision are marked for collision detection...
				if (ID1 != 0 && ID2 != 0)
				{
					//Remove the collision between these objects from the set of collision last step
					//If nothing was removed, then this is the start of a new collision!
					size_type collisionsRemoved = recordedCollisions.erase(collision);
					//Add a new version of this collision to the set for this step
					newCollisions.insert(collision);

					//Send a start collision callback if it is set and the collision is new
					if (collisionStartFunction != NULL && collisionsRemoved == 0)
					{
						collisionStartFunction(collision);
					}
					//Otherwise, the collision isn't new so send a collision contact callback
					else if (collisionCallbackFunction != NULL)
					{
						collisionCallbackFunction(collision);
					}
				}
			}
		}
	}

	//Any collisions that have been left in the previous tick's collision set...
	//Can't have happened again this tick, so send a collision end message
	for (auto itr = recordedCollisions.begin(); itr != recordedCollisions.end(); ++itr)
    {
        if (collisionEndFunction != NULL)
		{
			collisionEndFunction(*itr);
		}
    }
/*
	for each(PhysicsCollision collision in recordedCollisions)
	{
		if (collisionEndFunction != NULL)
		{
			collisionEndFunction(collision);
		}
	}
*/
	//The new set of collisions becomes the old set
	//The old set will be discarded when its nwe container goes out of scope
	recordedCollisions.swap(newCollisions);
}

/*
 * Find if a particular pair is colliding this step
 */
bool PhysicsWorld::areColliding(int ID1, int ID2)
{
	//Arbitrary normal and points, since these are not part of the hash function
	PhysicsCollision collisionQuery(ID1, ID2, glm::vec3(0.0f), glm::vec3(0.0f), glm::vec3(0.0f));

	std::unordered_set<PhysicsCollision, CollisionHasher>::const_iterator collision = recordedCollisions.find(collisionQuery);

	return collision != recordedCollisions.end();
}

void PhysicsWorld::addPhysicsObject(PhysicsObject* object)
{
	dynamicsWorld->addRigidBody(object->getRigidBody());

	rigidBodies.push_back(object->getRigidBody());
	//std::cout << "Rigidbodies contains n items: " << rigidBodies.size() << std::endl;
}

void PhysicsWorld::addGravityObject(GravityObject* object)
{
	dynamicsWorld->addRigidBody(object->getRigidBody());

	gravityObjects.push_back(object);
}

void PhysicsWorld::removePhysicsObject(PhysicsObject* object)
{
	dynamicsWorld->removeRigidBody(object->getRigidBody());

	rigidBodies.erase(std::remove(rigidBodies.begin(), rigidBodies.end(), object->getRigidBody()), rigidBodies.end());
}

bool PhysicsWorld::getUsingRadians() const
{
	return usingRadians;
}

//Register callback functions with the physics world
void PhysicsWorld::setCollisionFunction(PWCollisionCallback cf)
{
	collisionCallbackFunction = cf;
}

void PhysicsWorld::setCollisionStartFunction(PWCollisionCallback csf)
{
	collisionStartFunction = csf;
}

void PhysicsWorld::setCollisionEndFunction(PWCollisionCallback cef)
{
	collisionEndFunction = cef;
}
