#pragma once
#ifndef RIGID_BODY_H_
#define RIGID_BODY_H_
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <GLFW/glfw3.h>

#include "UtilFunc.h"

struct Contact;
class CarnegieRigidBody {
public:
	CarnegieRigidBody(float mass, float width, float height, float depth);
	void ComputeForceAndTorque(glm::vec3 force, glm::vec3 r);
	void Update(float dt);
	void PrintState();
	void PrintForceAndTorque();
	glm::vec3 pt_velocity(glm::vec3 p);
	bool colliding(Contact* c);
	void collision(Contact* c, double epsilon);
	void find_all_collisions(Contact contacts[], int ncontacts);

	//for a cube
	float width, height, depth;

	float mass;
	glm::mat3 Ibody, IbodyInv;

	glm::vec3 position;    // x(t)
	glm::mat3 rotation;    // R(t)
	glm::vec4 orientation;  // q(t)
	glm::vec3 linMomentum; // P(t)
	glm::vec3 angMomentum; // L(t)
	
	glm::mat3 I;
	glm::mat3 Iinv;        // changes whenever the body moves?
	glm::vec3 velocity;
	glm::vec3 angularVelocity;
	
	glm::vec3 force;
	glm::vec3 torque;

	glm::vec3 GetVertexInWorldSpace(int vertex);

	glm::vec3 vertices[8];
};
#endif