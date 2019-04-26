#include "RigidBody.h"
RigidBody::RigidBody(glm::vec2 initPos, float w, float h, float m) {
	this->position = initPos;
	this->linearVelocity = glm::vec2(0, 0);
	this->linearAcceleration = glm::vec2(0, 0);
	this->angle = 0;
	this->angularVelocity = 0;
	this->angularAcceleration = 0;
	this->mass = m;
	this->width = w;
	this->height = h;
	this->momentOfInertia = m * (h * h + w * w) / 12.0;
}

void RigidBody::ComputeAccels(glm::vec2 force, glm::vec2 r) {
	linearAcceleration += (force / mass);
	angularAcceleration += (r.x * force.y - r.y * force.x) / momentOfInertia;
}

void RigidBody::Update(float dt) {
	linearVelocity += linearAcceleration * dt;
	position += linearVelocity * dt;
	angularVelocity += angularAcceleration * dt;
	angle += angularVelocity * dt;
	linearAcceleration = glm::vec2(0, 0);
	angularAcceleration = 0;
}