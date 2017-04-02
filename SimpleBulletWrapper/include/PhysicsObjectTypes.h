#ifndef PHYSICS_OBJECT_TYPES
#define PHYSICS_OBJECT_TYPES

#include "PhysicsObject.h"
#include "HeightfieldData.h"
#include <BulletCollision\CollisionShapes\btHeightfieldTerrainShape.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <string>
#include <vector>

class PhysicsBox :public PhysicsObject
{
public:
	//Lesson 1 constructors: absolute minimum to set up a test scene
	PhysicsBox(PhysicsWorld* world);
	//Lesson 2: Trying out different initial conditions
	PhysicsBox(glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world);
	PhysicsBox(glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world);
	//Full-feature constructors
	PhysicsBox(bool dynamic, float depth, float height, float width, float mass, glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world);
	PhysicsBox(bool dynamic, float depth, float height, float width, float mass, glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world);
private:
	void construct(bool dynamic, float depth, float height, float width, float mass, glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world);
	void construct(bool dynamic, float depth, float height, float width, float mass, glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world);
};

class PhysicsPlane :public PhysicsObject
{
public:
	//Lesson 1: absolute minimum to set up a test scene
	PhysicsPlane(PhysicsWorld* world);
	//Full-feature constructor
	PhysicsPlane(glm::vec3 normal, float normalDistance, PhysicsWorld* world);
private:
	void construct(glm::vec3 normal, float normalDistance, PhysicsWorld* world);
};

class PhysicsHeightmap :public PhysicsObject
{
	HeightfieldData heightmapData;
public:
	PhysicsHeightmap(std::string filename, float depth, float width, float maxHeight, glm::vec3 position, PhysicsWorld* world);
	~PhysicsHeightmap();

	HeightfieldData getHeightMapData() const;
};

class PhysicsBall :public PhysicsObject
{
public:
	//Lesson 2: Trying out different initial conditions
	PhysicsBall(glm::vec3 initialPosition, PhysicsWorld* world);
	//Full-feature constructor
	PhysicsBall(bool dynamic, float radius, float mass, glm::vec3 initialPosition, PhysicsWorld* world);
private:
	void construct(bool dynamic, float radius, float mass, glm::vec3 initialPosition, PhysicsWorld* world);
};

class PhysicsCone :public PhysicsObject
{
public:
	//Lesson 2: Trying out different initial conditions
	PhysicsCone(glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world);
	PhysicsCone(glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world);
	//Full-feature constructors
	PhysicsCone(bool dynamic, float height, float baseRadius, float mass, glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world);
	PhysicsCone(bool dynamic, float height, float baseRadius, float mass, glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world);
private:
	void construct(bool dynamic, float height, float baseRadius, float mass, glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world);
	void construct(bool dynamic, float height, float baseRadius, float mass, glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world);
};

class PhysicsCylinder :public PhysicsObject
{
public:
	//Lesson 2: Trying out different initial conditions
	PhysicsCylinder(glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world);
	PhysicsCylinder(glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world);
	//Full-feature constructor
	PhysicsCylinder(bool dynamic, float depth, float height, float width, float mass, glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world);
	PhysicsCylinder(bool dynamic, float depth, float height, float width, float mass, glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world);
private:
	void construct(bool dynamic, float depth, float height, float width, float mass, glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world);
	void construct(bool dynamic, float depth, float height, float width, float mass, glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world);
};

class PhysicsCapsule :public PhysicsObject
{
public:
	//Lesson 2: Trying out different initial conditions
	PhysicsCapsule(glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world);
	PhysicsCapsule(glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world);
	//Full-feature constructor
	PhysicsCapsule(bool dynamic, float radius, float height, float mass, glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world);
	PhysicsCapsule(bool dynamic, float radius, float height, float mass, glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world);
private:
	void construct(bool dynamic, float radius, float height, float mass, glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world);
	void construct(bool dynamic, float radius, float height, float mass, glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world);
};

class PhysicsConvexMesh :public PhysicsObject
{
public:
	PhysicsConvexMesh(bool dynamic, std::string filename, float mass, glm::vec3 initialPosition, glm::vec3 initialOrientationXYZ, PhysicsWorld* world);
	PhysicsConvexMesh(bool dynamic, std::string filename, float mass, glm::vec3 initialPosition, glm::vec4 initialOrientationQuat, PhysicsWorld* world);
};

class PhysicsConcaveMesh :public PhysicsObject
{
	//Data has to be kept around for the duration of the object
	std::vector<float> vertices;
	btTriangleMesh triangleMesh;
public:
	PhysicsConcaveMesh(std::string filename, glm::vec3 position, glm::vec3 orientationXYZ, PhysicsWorld* world);
	PhysicsConcaveMesh(std::string filename, glm::vec3 position, glm::vec4 orientationQuat, PhysicsWorld* world);
};
#endif
