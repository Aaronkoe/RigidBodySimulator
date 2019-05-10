#pragma once
#include "RigidBody.h"

struct Contact {
public:
	CarnegieRigidBody * a;
	CarnegieRigidBody * b; // a contains rigid body whose vertex is in b's face
	glm::vec3 p, // worldspace position vector
		n, // normal of face, if vertex face collision
		ea, // direction of edge of a
		eb; // direction of edge of b
	bool vertexFace; // true if vertex face collision
};