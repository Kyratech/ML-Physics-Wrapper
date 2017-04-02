#ifndef PHYSICS_WRAPPER_OBJECT
#define PHYSICS_WRAPPER_OBJECT

//Physics
#include <btBulletDynamicsCommon.h>
#include "glm/glm.hpp"
#include "glm/gtx/quaternion.hpp"

class PhysicsWorld;

class PhysicsObject
{
private:
	btCollisionShape* collisionShape;
	btDefaultMotionState* motionState;
	btRigidBody* rigidBody;

	btTransform initialTransform;

	int collisionID;
protected:
	PhysicsObject() {};
public:
	void Init(btCollisionShape* cs, btDefaultMotionState* ms, btRigidBody* rb, PhysicsWorld* world);
	~PhysicsObject();

	btRigidBody* getRigidBody() const;
	//TODO: Copy constructor and copy assignment

	glm::vec3 getPosition() const;
	glm::quat getRotationQuaternion() const;

	void resetTransform();

	void applyForce(glm::vec3 force);
	void applyImpulse(glm::vec3 impulse);

	void setCollisionID(int ID);
	int getCollisionID();
};

#endif
