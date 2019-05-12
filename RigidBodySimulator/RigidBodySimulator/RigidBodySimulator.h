#pragma once
#ifndef RIGID_BODY_SIMULATOR_H_
#define RIGID_BODY_ISMULATOR_H_

#include <vector>
#include "RigidBody.h"
#include <learnopengl/shader_m.h>

struct Collision {
	Collision(RigidBody2D* a, RigidBody2D* b, int vertexColliding) {
		this->a = a; // vertex rigid body
		this->b = b;
		this->vertexThatIsColliding = vertexColliding;
	}
	RigidBody2D* a,* b;
	int vertexThatIsColliding;
};

class RigidBodySimulator {
public:
	std::vector<RigidBody2D*> rigidBodies;
	std::vector<RigidBody2D*> walls;
	std::vector<Collision> collisions;
	int collisionSteps = 5;

	void UpdateLoop(float dt);
	void AdvanceTime(float dt);
	void DrawBodies(Shader& shader, unsigned int vao);
	bool AreAnyCollisions();
	float AdvanceToCollisionDt(float dt);
	void HandleCollisions();
	void ApplyAcceleration(glm::vec2);
};

#endif