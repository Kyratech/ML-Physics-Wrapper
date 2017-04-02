#ifndef PHYSICS_COLLISION_H
#define PHYSICS_COLLISION_H

#include "glm/glm.hpp"
#include <functional>

class PhysicsCollision
{
public:
	//Constructors
	PhysicsCollision(int cID1, int cID2, glm::vec3 p1, glm::vec3 p2, glm::vec3 norm);

	//Accessors
	int getID1() const;
	int getID2() const;
	glm::vec3 getPos1() const;
	glm::vec3 getPos2() const;
	glm::vec3 getNormal() const;

	//Check to see if a particular ID was involved in the collision
	bool contains(int ID) const;
	//Check to see if a particular pair of IDs were involved
	bool contains(int ID1, int ID2) const;

	//Check equality
	bool operator ==(const PhysicsCollision & obj) const;
private:
	//Stores the collision ID of the two involved objects
	int id1, id2;
	//The point of contact in 3d space
	//Numbers correspond to the order of the collision IDs
	glm::vec3 loc1, loc2;
	//The normal of the contact
	glm::vec3 normal;
};

struct CollisionHasher
{
	size_t operator()(const PhysicsCollision & collision) const
	{
		return (std::hash<int>()(collision.getID1()) + std::hash<int>()(collision.getID2()));
	}
};

#endif