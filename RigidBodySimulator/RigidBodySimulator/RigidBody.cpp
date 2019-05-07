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

CarnegieRigidBody::CarnegieRigidBody(float mass, float width, float height, float depth)
{
	this->mass = mass;
	this->width = width;
	this->height = height;
	this->depth = depth;
	this->Ibody = glm::mat3(0.0);
	this->Ibody[0][0] = 1.0 / 12.0 * mass * (pow(height, 2) + pow(depth, 2));
	this->Ibody[1][1] = 1.0 / 12.0 * mass * (pow(width, 2) + pow(depth, 2));
	this->Ibody[2][2] = 1.0 / 12.0 * mass * (pow(height, 2) + pow(width, 2));
	this->IbodyInv = glm::inverse(Ibody);
	this->orientation = glm::vec4(1, 0, 0, 0);
	this->orientation = glm::normalize(orientation);
	this->rotation = glm::mat3(1.0);
	this->position = glm::vec3(0, 0, 0);
	this->linMomentum = glm::vec3(0, 0, 0);
	this->angMomentum = glm::vec3(0, 0, 0);
	this->Iinv = glm::mat3(1.0);
	this->velocity = glm::vec3(0, 0, 0);
	this->angularVelocity = glm::vec3(0, 0, 0);
	this->force = glm::vec3(0, 0, 0);
	this->torque = glm::vec3(0, 0, 0);
}

void CarnegieRigidBody::ComputeForceAndTorque(glm::vec3 force, glm::vec3 r) {
	this->force += force;
	this->torque += glm::cross(r, force);
}

void CarnegieRigidBody::Update(float dt) {
	this->velocity = linMomentum / mass;
	this->I = this->rotation * this->Ibody * glm::transpose(this->rotation);
	this->Iinv = glm::inverse(I);
	this->angularVelocity = Iinv * angMomentum;
	//glm::mat3 omegaStar = glm::mat3(0.0);
	//omegaStar[0][1] = -angularVelocity.z;
	//omegaStar[1][2] = angularVelocity.y;
	//omegaStar[1][0] = angularVelocity.z;
	//omegaStar[1][2] = -angularVelocity.x;
	//omegaStar[2][0] = -angularVelocity.y;
	//omegaStar[2][1] = angularVelocity.x;
	//omega orientation
	glm::vec3 v1 = angularVelocity;
	glm::vec3 v2 = glm::vec3(orientation[1], orientation[2], orientation[3]);
	float s2 = orientation[0];
	glm::vec4 quatProd = glm::vec4(glm::dot(v1, v2), s2 * v1 + glm::cross(v1, v2));

	this->position += velocity * dt;
	this->orientation += .5f * (quatProd)* dt;
	//this->rotation += omegaStar * rotation;
	this->linMomentum += force * dt;
	this->angMomentum += torque * dt;

	//std::cout << "det rotation =" << glm::determinant(rotation) << std::endl;
	this->force = glm::vec3(0, 0, 0);
	this->torque = glm::vec3(0, 0, 0);
}

void CarnegieRigidBody::PrintState()
{
	std::cout << "Pos: "; PrintGlmVec3(position);
	std::cout << "Rot: \n"; PrintGlmMat3(rotation);
	std::cout << "lin Mom: "; PrintGlmVec3(linMomentum);
	std::cout << "ang Mom: "; PrintGlmVec3(angMomentum);
}

void CarnegieRigidBody::PrintForceAndTorque()
{
	std::cout << "Force: "; PrintGlmVec3(force);
	std::cout << "Torque: "; PrintGlmVec3(torque);
}

glm::vec4 CarnegieRigidBody::GetAngleAndAxis()
{
	float angle = 2 * acos(orientation[0]);
	if (angle > .00001) {
		glm::vec3 axis = glm::vec3(orientation[1], orientation[2], orientation[3]) / angle;
		return glm::vec4(angle, axis.x, axis.y, axis.z);
	}
	return glm::vec4(angle, 1, 1, 1);
}
