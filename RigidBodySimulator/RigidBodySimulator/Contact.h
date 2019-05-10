#pragma once
#ifndef CONTACT_H_
#define CONTACT_H_
#include "RigidBody.h"

struct Contact {
	Contact() = default;
	CarnegieRigidBody* a, * b; // a contains rigid body whose vertex is in b's face
	glm::vec3 p, // worldspace position vector
		n, // normal of face, if vertex face collision
		ea, // direction of edge of a
		eb; // direction of edge of b
	bool vertexFace; // true if vertex face collision
};


#endif