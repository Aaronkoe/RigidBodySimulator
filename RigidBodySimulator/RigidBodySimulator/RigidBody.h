#pragma once
#include <glm/glm.hpp>
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

	void ComputeAccels(glm::vec2 force, glm::vec2 r);
	void Update(float dt);
};