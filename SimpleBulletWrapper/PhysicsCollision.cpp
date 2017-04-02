#include "include/PhysicsCollision.h"

PhysicsCollision::PhysicsCollision(int cID1, int cID2, glm::vec3 p1, glm::vec3 p2, glm::vec3 norm)
{
	id1 = cID1;
	id2 = cID2;
	loc1 = p1;
	loc2 = p2;
	normal = norm;
}

//Access the IDs of the objects involved in the collision
int PhysicsCollision::getID1() const
{
	return id1;
}

int PhysicsCollision::getID2() const
{
	return id2;
}

//Access the points of the objects involved in the collision
glm::vec3 PhysicsCollision::getPos1() const
{
	return loc1;
}

glm::vec3 PhysicsCollision::getPos2() const
{
	return loc2;
}

//Get the normal of the collision
//This is useful for stuff like firing debris out in the direction of the collision
glm::vec3 PhysicsCollision::getNormal() const
{
	return normal;
}

//Check to see if a particular collision ID is involved in the collision
bool PhysicsCollision::contains(int ID) const
{
	if (id1 == ID || id2 == ID)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool PhysicsCollision::contains(int ID1, int ID2) const
{
	if (contains(ID1) && contains(ID2))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*
 * Two collisions are equal if they involve the same pair of objects
*/
bool PhysicsCollision::operator==(const PhysicsCollision & obj) const
{
	if (obj.contains(id1, id2))
	{
		return true;
	}
	else
	{
		return false;
	}
}
