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

void RigidBody::ComputeAccels(glm::vec2 force, glm::vec2 r, int cubeVao) {
	linearAcceleration += (force / mass);
	angularAcceleration += (r.x * force.y - r.y * force.x) / momentOfInertia;
	glBindVertexArray(cubeVao);
	glDrawArrays(GL_TRIANGLES, 0, 36);
}

void RigidBody::Update(float dt) {
	linearVelocity += linearAcceleration * dt;
	position += linearVelocity * dt;
	angularVelocity += angularAcceleration * dt;
	angle += angularVelocity * dt;
	linearAcceleration = glm::vec2(0, 0);
	angularAcceleration = 0;
	glm::mat4 model = glm::mat4(1.0f);
}

void CarnegieRigidBody::ComputeForceAndTorque(glm::vec3 force, glm::vec3 r) {
	this->force += force;
}

void CarnegieRigidBody::Update(float dt) {
	glm::vec3 accel = force / mass;
}

CarnegieRigidBody::CarnegieRigidBody(float mass, glm::mat3 IbodyIn) {
	this->mass = mass;
	this->Ibody = IbodyIn;
	this->IbodyInv = glm::inverse(IbodyIn);
}