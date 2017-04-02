#include "include/PhysicsObject.h"
#include "include/PhysicsWorld.h"

void PhysicsObject::Init(btCollisionShape* cs, btDefaultMotionState* ms, btRigidBody* rb, PhysicsWorld* world)
{
	collisionShape = cs;
	motionState = ms;
	rigidBody = rb;

	//Store the intitial transform so we can reset the object later if need be
	rigidBody->getMotionState()->getWorldTransform(initialTransform);

	/* Set the collisionID to default of 0. Unless changed, this object will not report collisions.
	 * With the exception of other !=0 objects colliding with it.
	 */
	rigidBody->setUserPointer(0);
}

PhysicsObject::~PhysicsObject()
{
	delete rigidBody;
	delete motionState;
	delete collisionShape;
}

glm::vec3 PhysicsObject::getPosition() const
{
	btTransform transform;
	this->rigidBody->getMotionState()->getWorldTransform(transform);
	btVector3 origin = transform.getOrigin();
	glm::vec3 position(origin.x(), origin.y(), origin.z());
	return position;
}

glm::quat PhysicsObject::getRotationQuaternion() const
{
	btTransform transform;
	this->rigidBody->getMotionState()->getWorldTransform(transform);
	btQuaternion rotation = transform.getRotation();
	glm::quat rotationQuat = glm::angleAxis(rotation.getAngle(), glm::vec3(rotation.getAxis().getX(), rotation.getAxis().getY(), rotation.getAxis().getZ()));
	return rotationQuat;
}

btRigidBody* PhysicsObject::getRigidBody() const
{
	return rigidBody;
}

void PhysicsObject::resetTransform()
{
	rigidBody->clearForces();

	//Stop the object moving
	btVector3 zeroVector(0, 0, 0);
	rigidBody->setLinearVelocity(zeroVector);
	rigidBody->setAngularVelocity(zeroVector);

	//Teleport the object to its initial position
	rigidBody->setCenterOfMassTransform(initialTransform);

	//If the object had come to rest, it may have deactivated. Reactivate it so that the world starts updating it again.
	rigidBody->activate();
}

void PhysicsObject::applyForce(glm::vec3 force)
{
	rigidBody->applyCentralForce(btVector3(force.x, force.y, force.z));
}

void PhysicsObject::applyImpulse(glm::vec3 impulse)
{
	rigidBody->applyCentralImpulse(btVector3(impulse.x, impulse.y, impulse.z));
}

/* Access the Collision ID. Used to tag objects as a certain type of object.
 * When reading collisions, the user can use the IDs of the objects returned to determine behaviour
 * Each object could have a unique ID, or objects that behave the same can share IDs.
 * ID of 0 indicates that we're not interested in these objects - no collision response sent.
 */
void PhysicsObject::setCollisionID(int ID)
{
	collisionID = ID;
	rigidBody->setUserPointer((void*) ID);
}

int PhysicsObject::getCollisionID()
{
	return collisionID;
}