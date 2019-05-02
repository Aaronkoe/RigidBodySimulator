#pragma once
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <GLFW/glfw3.h>
class RigidBody {
public:
	RigidBody(glm::vec2 initPos, float w, float h, float m);
	glm::vec2 position;
	glm::vec2 linearVelocity;
	glm::vec2 linearAcceleration;
	float angle;
	float angularVelocity;
	float angularAcceleration;
	float momentOfInertia;
	float mass;
	float width, height;

	void ComputeAccels(glm::vec2 force, glm::vec2 r, int cubeVao);
	void Update(float dt);
};

// rigid body from carnegie
class CarnegieRigidBody {
public:
	CarnegieRigidBody(float mass, glm::mat3 IbodyIn);
	void ComputeForceAndTorque(glm::vec3 force, glm::vec3 r);
	void Update(float dt);

	float mass;
	glm::mat3 Ibody, IbodyInv;

	glm::vec3 position; // x(t)
	glm::mat3 rotation; // R(t)
	glm::vec3 linMomentum; // P(t)
	glm::vec3 angMomentum; // L(t)
	
	glm::mat3 Iinv;
	glm::vec3 velocity;
	glm::vec3 angularVelocity;
	
	glm::vec3 force;
	glm::vec3 torque;
};