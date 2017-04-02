#include "include/PhysicsObjectTypes.h"
#include "include/PhysicsUtilities.h"
#include "include/PhysicsWorld.h"

//======//
// CUBE //
//======//

/* Unit cube, at 10 units above the origin*/
PhysicsBox::PhysicsBox(PhysicsWorld* world)
{
	construct(true, 1.0f, 1.0f, 1.0f, 1.0f, glm::vec3(0.0f, 10.0f, 0.0f), glm::vec3(0.0f), world);
}

/* Unit cube */
PhysicsBox::PhysicsBox(glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world)
{
	construct(true, 1.0f, 1.0f, 1.0f, 1.0f, initialPosition, initialOrientationXYZ, world);
}

PhysicsBox::PhysicsBox(glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world)
{
	construct(true, 1.0f, 1.0f, 1.0f, 1.0f, initialPosition, initialOrientationQuat, world);
}

PhysicsBox::PhysicsBox(bool dynamic, float depth, float height, float width, float mass, glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world)
{
	construct(dynamic, depth, height, width, mass, initialPosition, initialOrientationXYZ, world);
}

PhysicsBox::PhysicsBox(bool dynamic, float depth, float height, float width, float mass, glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world)
{
	construct(dynamic, depth, height, width, mass, initialPosition, initialOrientationQuat, world);
}

/*
 * Width, height and depth are represented by the engine's Z, Y and X directions respectively
*/
void PhysicsBox::construct(bool dynamic, float depth, float height, float width, float mass, glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world)
{
	btCollisionShape* cShape = new btBoxShape(btVector3(depth / 2.0, height / 2.0, width / 2.0));

	//Convert values to radians if not already
	if (!world->getUsingRadians())
	{
		initialOrientationXYZ = glm::vec3(glm::radians(initialOrientationXYZ.x), glm::radians(initialOrientationXYZ.y), glm::radians(initialOrientationXYZ.z));
	}
	glm::vec4 rotationQuat = PhysicsUtilities::EulerToQuaternion(initialOrientationXYZ);

	btDefaultMotionState* mState = new btDefaultMotionState(btTransform(btQuaternion(rotationQuat.w, rotationQuat.x, rotationQuat.y, rotationQuat.z), btVector3(initialPosition.x, initialPosition.y, initialPosition.z)));

	btRigidBody* rBody = PhysicsUtilities::constructRigidBody(cShape, mState, dynamic, mass);

	//IMPORTANT: Store the physics data structures in the superclass fields!
	Init(cShape, mState, rBody, world);
	//Adding the object to a world will allow it to be simulated when stepping the world
	world->addPhysicsObject(this);
}

void PhysicsBox::construct(bool dynamic, float depth, float height, float width, float mass, glm::vec3 initialPosition, glm::vec4 inititalOrientationQuat, PhysicsWorld* world)
{
	btCollisionShape* cShape = new btBoxShape(btVector3(depth / 2.0, height / 2.0, width / 2.0));

	//Convert values to radians if not already
	if (!world->getUsingRadians())
	{
		inititalOrientationQuat.w = glm::radians(inititalOrientationQuat.w);
	}

	btDefaultMotionState* mState = new btDefaultMotionState(btTransform(btQuaternion(inititalOrientationQuat.w, inititalOrientationQuat.x, inititalOrientationQuat.y, inititalOrientationQuat.z), btVector3(initialPosition.x, initialPosition.y, initialPosition.z)));

	btRigidBody* rBody = PhysicsUtilities::constructRigidBody(cShape, mState, dynamic, mass);

	//IMPORTANT: Store the physics data structures in the superclass fields!
	Init(cShape, mState, rBody, world);
	//Adding the object to a world will allow it to be simulated when stepping the world
	world->addPhysicsObject(this);
}

//=======//
// Plane //
//=======//
PhysicsPlane::PhysicsPlane(PhysicsWorld* world)
{
	construct(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f, world);
}

PhysicsPlane::PhysicsPlane(glm::vec3 normal, float normalDistance, PhysicsWorld* world)
{
	construct(normal, normalDistance, world);
}

void PhysicsPlane::construct(glm::vec3 normal, float normalDistance, PhysicsWorld* world)
{
	btCollisionShape* cShape = new btStaticPlaneShape(btVector3(normal.x, normal.y, normal.z), normalDistance);

	btDefaultMotionState* mState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));

	//This is the construction info for a non-dynamic object: moves under NO circumstance
	btRigidBody::btRigidBodyConstructionInfo cInfo(0, mState, cShape, btVector3(0, 0, 0));

	btRigidBody* rBody = new btRigidBody(cInfo);

	//IMPORTANT: Store the physics data structures in the superclass fields!
	Init(cShape, mState, rBody, world);
	//Adding the object to a world will allow it to be simulated when stepping the world
	world->addPhysicsObject(this);
}

//===========//
// Heightmap //
//===========//
PhysicsHeightmap::PhysicsHeightmap(std::string filename, float depth, float width, float maxHeight, glm::vec3 position, PhysicsWorld* world)
{
	heightmapData = PhysicsUtilities::readImageIntoVector(filename, maxHeight);

	std::vector<float>* data = heightmapData.getData();
	btCollisionShape* cShape = new btHeightfieldTerrainShape(heightmapData.getDepth(), heightmapData.getWidth(), &(data->front()), 1.0, 0, maxHeight, 1, PHY_FLOAT,true);
	//Scale the heightmap to the desired world dimensions
	float depthScaleFactor = depth / heightmapData.getDepth();
	float widthScaleFactor = width / heightmapData.getWidth();
	btVector3 scalingXZ(depthScaleFactor, 1, widthScaleFactor);
	cShape->setLocalScaling(scalingXZ);

	btDefaultMotionState* mState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(position.x, position.y, position.z)));

	//This is the construction info for a non-dynamic object: moves under NO circumstance
	btRigidBody::btRigidBodyConstructionInfo cInfo(0, mState, cShape, btVector3(0, 0, 0));

	btRigidBody* rBody = new btRigidBody(cInfo);

	//IMPORTANT: Store the physics data structures in the superclass fields!
	Init(cShape, mState, rBody, world);
	//Adding the object to a world will allow it to be simulated when stepping the world
	world->addPhysicsObject(this);
}

PhysicsHeightmap::~PhysicsHeightmap()
{
	heightmapData.deleteData();
}

HeightfieldData PhysicsHeightmap::getHeightMapData() const
{
	return heightmapData;
}

//===============//
// Ball (sphere) //
//===============//

PhysicsBall::PhysicsBall(glm::vec3 initialPosition, PhysicsWorld* world)
{
	//Density of 1kg/unit^3
	float mass = 4.0f * 3.14159 * 0.5 * 0.5 * 0.5 / 3;
	construct(true, 0.5, mass, initialPosition, world);
}

/*
 * Creates a sphere centred on the initial position
 * Rotation irrelevent
*/
PhysicsBall::PhysicsBall(bool dynamic, float radius, float mass, glm::vec3 initialPosition, PhysicsWorld* world)
{
	construct(dynamic, radius, mass, initialPosition, world);
}

void PhysicsBall::construct(bool dynamic, float radius, float mass, glm::vec3 initialPosition, PhysicsWorld* world)
{
	btCollisionShape* cShape = new btSphereShape(radius);

	btDefaultMotionState* mState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(initialPosition.x, initialPosition.y, initialPosition.z)));

	btRigidBody* rBody = PhysicsUtilities::constructRigidBody(cShape, mState, dynamic, mass);

	//IMPORTANT: Store the physics data structures in the superclass fields!
	Init(cShape, mState, rBody, world);
	//Adding the object to a world will allow it to be simulated when stepping the world
	world->addPhysicsObject(this);
}

//======//
// Cone //
//======//
//Unit height, unit diameter
PhysicsCone::PhysicsCone(glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world)
{
	//Density of 1kg/unit^3
	float mass = 3.14159f * 0.5 * 0.5 / 3;
	construct(true, 1.0f, 0.5f, mass, initialPosition, initialOrientationXYZ, world);
}

PhysicsCone::PhysicsCone(glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world)
{
	//Density of 1kg/unit^3
	float mass = 3.14159f * 0.5 * 0.5 / 3;
	construct(true, 1.0f, 0.5f, mass, initialPosition, initialOrientationQuat, world);
}

/*
 * Creates a cone centred around the y-axis (when supplied rotation is 0)
 * Tip-down with no rotation
 */
PhysicsCone::PhysicsCone(bool dynamic, float height, float baseRadius, float mass, glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world)
{
	construct(dynamic, height, baseRadius, mass, initialPosition, initialOrientationXYZ, world);
}

PhysicsCone::PhysicsCone(bool dynamic, float height, float baseRadius, float mass, glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world)
{
	construct(dynamic, height, baseRadius, mass, initialPosition, initialOrientationQuat, world);
}

void PhysicsCone::construct(bool dynamic, float height, float baseRadius, float mass, glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world)
{
	btCollisionShape* cShape = new btConeShape(baseRadius, height);

	//Convert values to radians if not already
	if (!world->getUsingRadians())
	{
		initialOrientationXYZ = glm::vec3(glm::radians(initialOrientationXYZ.x), glm::radians(initialOrientationXYZ.y), glm::radians(initialOrientationXYZ.z));
	}
	glm::vec4 rotationQuat = PhysicsUtilities::EulerToQuaternion(initialOrientationXYZ);

	btDefaultMotionState* mState = new btDefaultMotionState(btTransform(btQuaternion(rotationQuat.w, rotationQuat.x, rotationQuat.y, rotationQuat.z), btVector3(initialPosition.x, initialPosition.y, initialPosition.z)));

	btRigidBody* rBody = PhysicsUtilities::constructRigidBody(cShape, mState, dynamic, mass);

	//IMPORTANT: Store the physics data structures in the superclass fields!
	Init(cShape, mState, rBody, world);
	//Adding the object to a world will allow it to be simulated when stepping the world
	world->addPhysicsObject(this);
}

void PhysicsCone::construct(bool dynamic, float height, float baseRadius, float mass, glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world)
{
	btCollisionShape* cShape = new btConeShape(baseRadius, height);

	//Convert values to radians if not already
	if (!world->getUsingRadians())
	{
		initialOrientationQuat.w = glm::radians(initialOrientationQuat.w);
	}

	btDefaultMotionState* mState = new btDefaultMotionState(btTransform(btQuaternion(initialOrientationQuat.w, initialOrientationQuat.x, initialOrientationQuat.y, initialOrientationQuat.z), btVector3(initialPosition.x, initialPosition.y, initialPosition.z)));

	btRigidBody* rBody = PhysicsUtilities::constructRigidBody(cShape, mState, dynamic, mass);

	//IMPORTANT: Store the physics data structures in the superclass fields!
	Init(cShape, mState, rBody, world);
	//Adding the object to a world will allow it to be simulated when stepping the world
	world->addPhysicsObject(this);
}

//==========//
// Cylinder //
//==========//
//Base diameter = 1, height = 1
PhysicsCylinder::PhysicsCylinder(glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world)
{
	//Density of 1kg/unit^3
	float mass = 3.14159f * 0.5 * 0.5;
	construct(true, 1.0, 1.0, 1.0, mass, initialPosition, initialOrientationXYZ, world);
}

PhysicsCylinder::PhysicsCylinder(glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world)
{
	//Density of 1kg/unit^3
	float mass = 3.14159f * 0.5 * 0.5;
	construct(true, 1.0, 1.0, 1.0, mass, initialPosition, initialOrientationQuat, world);
}

/*
* Creates a cylinder centred around the y-axis (when supplied rotation is 0)
* Doesn't require cross section to be circular, so provide XYZ dimensions instead of radius
*/
PhysicsCylinder::PhysicsCylinder(bool dynamic, float depth, float height, float width, float mass, glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world)
{
	construct(dynamic, depth, height, width, mass, initialPosition, initialOrientationXYZ, world);
}

PhysicsCylinder::PhysicsCylinder(bool dynamic, float depth, float height, float width, float mass, glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world)
{
	construct(dynamic, depth, height, width, mass, initialPosition, initialOrientationQuat, world);
}

void PhysicsCylinder::construct(bool dynamic, float depth, float height, float width, float mass, glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world)
{
	btCollisionShape* cShape = new btCylinderShape(btVector3(depth / 2.0, height / 2.0, width / 2.0));

	//Convert values to radians if not already
	if (!world->getUsingRadians())
	{
		initialOrientationXYZ = glm::vec3(glm::radians(initialOrientationXYZ.x), glm::radians(initialOrientationXYZ.y), glm::radians(initialOrientationXYZ.z));
	}
	glm::vec4 rotationQuat = PhysicsUtilities::EulerToQuaternion(initialOrientationXYZ);

	btDefaultMotionState* mState = new btDefaultMotionState(btTransform(btQuaternion(rotationQuat.w, rotationQuat.x, rotationQuat.y, rotationQuat.z), btVector3(initialPosition.x, initialPosition.y, initialPosition.z)));

	btRigidBody* rBody = PhysicsUtilities::constructRigidBody(cShape, mState, dynamic, mass);

	//IMPORTANT: Store the physics data structures in the superclass fields!
	Init(cShape, mState, rBody, world);
	//Adding the object to a world will allow it to be simulated when stepping the world
	world->addPhysicsObject(this);
}

void PhysicsCylinder::construct(bool dynamic, float depth, float height, float width, float mass, glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world)
{
	btCollisionShape* cShape = new btCylinderShape(btVector3(depth / 2.0, height / 2.0, width / 2.0));

	//Convert values to radians if not already
	if (!world->getUsingRadians())
	{
		initialOrientationQuat.w = glm::radians(initialOrientationQuat.w);
	}

	btDefaultMotionState* mState = new btDefaultMotionState(btTransform(btQuaternion(initialOrientationQuat.w, initialOrientationQuat.x, initialOrientationQuat.y, initialOrientationQuat.z), btVector3(initialPosition.x, initialPosition.y, initialPosition.z)));

	btRigidBody* rBody = PhysicsUtilities::constructRigidBody(cShape, mState, dynamic, mass);

	//IMPORTANT: Store the physics data structures in the superclass fields!
	Init(cShape, mState, rBody, world);
	//Adding the object to a world will allow it to be simulated when stepping the world
	world->addPhysicsObject(this);
}

//=========//
// Capsule //
//=========//

//Radius = 0.5, height = 1 (total height therefore is 2)
PhysicsCapsule::PhysicsCapsule(glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world)
{
	float endSphereHalvesMass = 4.0f * 3.14159 * 0.5 * 0.5 * 0.5 / 3;
	float middleCylinderMass = 3.14159f * 0.5 * 0.5;
	float mass = endSphereHalvesMass + middleCylinderMass;
	construct(true, 0.5f, 1.0f, mass, initialPosition, initialOrientationXYZ, world);
}

PhysicsCapsule::PhysicsCapsule(glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world)
{
	float endSphereHalvesMass = 4.0f * 3.14159 * 0.5 * 0.5 * 0.5 / 3;
	float middleCylinderMass = 3.14159f * 0.5 * 0.5;
	float mass = endSphereHalvesMass + middleCylinderMass;
	construct(true, 0.5f, 1.0f, mass, initialPosition, initialOrientationQuat, world);
}

/*
 * Create a capsule centred around the y-axis.
 * Has circular cross-section. 
 */
PhysicsCapsule::PhysicsCapsule(bool dynamic, float radius, float height, float mass, glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world)
{
	construct(dynamic, radius, height, mass, initialPosition, initialOrientationXYZ, world);
}

PhysicsCapsule::PhysicsCapsule(bool dynamic, float radius, float height, float mass, glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world)
{
	construct(dynamic, radius, height, mass, initialPosition, initialOrientationQuat, world);
}

void PhysicsCapsule::construct(bool dynamic, float radius, float height, float mass, glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world)
{
	btCollisionShape* cShape = new btCapsuleShape(radius, height);

	//Convert values to radians if not already
	if (!world->getUsingRadians())
	{
		initialOrientationXYZ = glm::vec3(glm::radians(initialOrientationXYZ.x), glm::radians(initialOrientationXYZ.y), glm::radians(initialOrientationXYZ.z));
	}
	glm::vec4 rotationQuat = PhysicsUtilities::EulerToQuaternion(initialOrientationXYZ);

	btDefaultMotionState* mState = new btDefaultMotionState(btTransform(btQuaternion(rotationQuat.w, rotationQuat.x, rotationQuat.y, rotationQuat.z), btVector3(initialPosition.x, initialPosition.y, initialPosition.z)));

	btRigidBody* rBody = PhysicsUtilities::constructRigidBody(cShape, mState, dynamic, mass);

	//IMPORTANT: Store the physics data structures in the superclass fields!
	Init(cShape, mState, rBody, world);
	//Adding the object to a world will allow it to be simulated when stepping the world
	world->addPhysicsObject(this);
}

void PhysicsCapsule::construct(bool dynamic, float radius, float height, float mass, glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world)
{
	btCollisionShape* cShape = new btCapsuleShape(radius, height);

	//Convert values to radians if not already
	if (!world->getUsingRadians())
	{
		initialOrientationQuat.w = glm::radians(initialOrientationQuat.w);
	}

	btDefaultMotionState* mState = new btDefaultMotionState(btTransform(btQuaternion(initialOrientationQuat.w, initialOrientationQuat.x, initialOrientationQuat.y, initialOrientationQuat.z), btVector3(initialPosition.x, initialPosition.y, initialPosition.z)));

	btRigidBody* rBody = PhysicsUtilities::constructRigidBody(cShape, mState, dynamic, mass);

	//IMPORTANT: Store the physics data structures in the superclass fields!
	Init(cShape, mState, rBody, world);
	//Adding the object to a world will allow it to be simulated when stepping the world
	world->addPhysicsObject(this);
}

//=============//
// Convex mesh //
//=============//

PhysicsConvexMesh::PhysicsConvexMesh(bool dynamic, std::string filename, float mass, glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world)
{
	std::vector<float> vertices = PhysicsUtilities::readOBJIntoVector(filename);

	btCollisionShape* cShape = new btConvexHullShape(&vertices[0], vertices.size() / 3, 3 * sizeof(float));

	//Convert values to radians if not already
	if (!world->getUsingRadians())
	{
		initialOrientationXYZ = glm::vec3(glm::radians(initialOrientationXYZ.x), glm::radians(initialOrientationXYZ.y), glm::radians(initialOrientationXYZ.z));
	}
	glm::vec4 rotationQuat = PhysicsUtilities::EulerToQuaternion(initialOrientationXYZ);

	btDefaultMotionState* mState = new btDefaultMotionState(btTransform(btQuaternion(rotationQuat.w, rotationQuat.x, rotationQuat.y, rotationQuat.z), btVector3(initialPosition.x, initialPosition.y, initialPosition.z)));

	btRigidBody* rBody = PhysicsUtilities::constructRigidBody(cShape, mState, dynamic, mass);

	//IMPORTANT: Store the physics data structures in the superclass fields!
	Init(cShape, mState, rBody, world);
	//Adding the object to a world will allow it to be simulated when stepping the world
	world->addPhysicsObject(this);
}

PhysicsConvexMesh::PhysicsConvexMesh(bool dynamic, std::string filename, float mass, glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world)
{
	std::vector<float> vertices = PhysicsUtilities::readOBJIntoVector(filename);

	btCollisionShape* cShape = new btConvexHullShape(&vertices[0], vertices.size() / 3, 3 * sizeof(float));

	//Convert values to radians if not already
	if (!world->getUsingRadians())
	{
		initialOrientationQuat.w = glm::radians(initialOrientationQuat.w);
	}

	btDefaultMotionState* mState = new btDefaultMotionState(btTransform(btQuaternion(initialOrientationQuat.w, initialOrientationQuat.x, initialOrientationQuat.y, initialOrientationQuat.z), btVector3(initialPosition.x, initialPosition.y, initialPosition.z)));

	btRigidBody* rBody = PhysicsUtilities::constructRigidBody(cShape, mState, dynamic, mass);

	//IMPORTANT: Store the physics data structures in the superclass fields!
	Init(cShape, mState, rBody, world);
	//Adding the object to a world will allow it to be simulated when stepping the world
	world->addPhysicsObject(this);
}

//=====================//
// Static condave mesh //
//=====================//

PhysicsConcaveMesh::PhysicsConcaveMesh(std::string filename, glm::vec3 position, glm::vec3 orientationXYZ, PhysicsWorld* world)
{
	vertices = PhysicsUtilities::readOBJIntoVector(filename);

	//Read the vertices into the triangle mesh
	triangleMesh = btTriangleMesh(true, false);
	for (int i = 0; i < vertices.size(); i += 9)
	{
		btVector3 v1(vertices[i], vertices[i + 1], vertices[i + 2]);
		btVector3 v2(vertices[i + 3], vertices[i + 4], vertices[i + 5]);
		btVector3 v3(vertices[i + 6], vertices[i + 7], vertices[i + 8]);
		triangleMesh.addTriangle(v1, v2, v3);
	}

	btCollisionShape* cShape = new btBvhTriangleMeshShape(&triangleMesh, false);

	//Convert values to radians if not already
	if (!world->getUsingRadians())
	{
		orientationXYZ = glm::vec3(glm::radians(orientationXYZ.x), glm::radians(orientationXYZ.y), glm::radians(orientationXYZ.z));
	}
	glm::vec4 rotationQuat = PhysicsUtilities::EulerToQuaternion(orientationXYZ);

	btDefaultMotionState* mState = new btDefaultMotionState(btTransform(btQuaternion(rotationQuat.w, rotationQuat.x, rotationQuat.y, rotationQuat.z), btVector3(position.x, position.y, position.z)));

	btRigidBody* rBody = PhysicsUtilities::constructRigidBody(cShape, mState, false, 0);

	//IMPORTANT: Store the physics data structures in the superclass fields!
	Init(cShape, mState, rBody, world);
	//Adding the object to a world will allow it to be simulated when stepping the world
	world->addPhysicsObject(this);
}

PhysicsConcaveMesh::PhysicsConcaveMesh(std::string filename, glm::vec3 position, glm::vec4 orientationQuat, PhysicsWorld* world)
{
	vertices = PhysicsUtilities::readOBJIntoVector(filename);

	//Read the vertices into the triangle mesh
	triangleMesh = btTriangleMesh(true, false);
	for (int i = 0; i < vertices.size(); i += 9)
	{
		btVector3 v1(vertices[i], vertices[i + 1], vertices[i + 2]);
		btVector3 v2(vertices[i + 3], vertices[i + 4], vertices[i + 5]);
		btVector3 v3(vertices[i + 6], vertices[i + 7], vertices[i + 8]);
		triangleMesh.addTriangle(v1, v2, v3);
	}

	btCollisionShape* cShape = new btBvhTriangleMeshShape(&triangleMesh, false);

	//Convert values to radians if not already
	if (!world->getUsingRadians())
	{
		orientationQuat.w = glm::radians(orientationQuat.w);
	}

	btDefaultMotionState* mState = new btDefaultMotionState(btTransform(btQuaternion(orientationQuat.w, orientationQuat.x, orientationQuat.y, orientationQuat.z), btVector3(position.x, position.y, position.z)));

	btRigidBody* rBody = PhysicsUtilities::constructRigidBody(cShape, mState, false, 0);

	//IMPORTANT: Store the physics data structures in the superclass fields!
	Init(cShape, mState, rBody, world);
	//Adding the object to a world will allow it to be simulated when stepping the world
	world->addPhysicsObject(this);
}
