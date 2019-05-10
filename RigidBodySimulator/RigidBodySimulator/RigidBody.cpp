#include "RigidBody.h"
#include "Contact.h"
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

	vertices[0] = glm::vec3(width / 2, height / 2, depth / 2);
	vertices[1] = glm::vec3(width / 2, height / 2, -depth / 2);
	vertices[2] = glm::vec3(width / 2, -height / 2, depth / 2);
	vertices[3] = glm::vec3(width / 2, -height / 2, -depth / 2);
	vertices[4] = glm::vec3(-width / 2, -height / 2, depth / 2);
	vertices[5] = glm::vec3(-width / 2, height / 2, -depth / 2);
	vertices[6] = glm::vec3(-width / 2, -height / 2, depth / 2);
	vertices[7] = glm::vec3(-width / 2, -height / 2, -depth / 2);
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
	glm::vec3 v1 = angularVelocity;
	glm::vec3 v2 = glm::vec3(orientation[1], orientation[2], orientation[3]);
	float s2 = orientation[0];
	glm::vec4 quatProd = glm::vec4(glm::dot(v1, v2), s2 * v1 + glm::cross(v1, v2));

	this->position += velocity * dt;
	this->orientation += .5f * (quatProd)* dt;
	this->linMomentum += force * dt;
	this->angMomentum += torque * dt;

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

//velocity of a point on a rigid body
glm::vec3 CarnegieRigidBody::pt_velocity(glm::vec3 p)
{
	return velocity + (glm::cross(angularVelocity, (p - position)));
}
const double THRESHOLD = .1;
bool CarnegieRigidBody::colliding(Contact* c)
{
	glm::vec3 padot = c->a->pt_velocity(c->p);
	glm::vec3 pbdot = c->b->pt_velocity(c->p);
	double vrel = glm::dot(c->n, (padot - pbdot));
	if (vrel > THRESHOLD) {
		return FALSE;
	}
	if (vrel > -THRESHOLD) {
		return FALSE;
	}
	else {
		return TRUE;
	}
}

glm::vec3 CarnegieRigidBody::GetVertexInWorldSpace(int vertex)
{
	glm::mat3 rotationMatrix = glm::toMat3(glm::quat(orientation[1], orientation[2], orientation[3], orientation[0]));
	return rotationMatrix * vertices[vertex] + position;
}

void collision(Contact* c, double epsilon)
{
	glm::vec3 padot = c->a->pt_velocity(c->p);
	glm::vec3 pbdot = c->b->pt_velocity(c->p);
	glm::vec3 n = c->n,
		ra = c->p - c->a->position,
		rb = c->p - c->b->position;
	double vrel = glm::dot(n, (padot - pbdot)),
		numerator = -(1 + epsilon) * vrel;
	double term1 = 1 / c->a->mass,
		term2 = 1 / c->b->mass,
		term3 = glm::dot( n, ((c->a->Iinv * glm::cross((glm::cross(ra, n)), ra)))),
		term4 = glm::dot(n, ((c->b->Iinv * glm::cross((glm::cross(rb, n)), rb))));
	float j = numerator / (term1 + term2 + term3 + term4);
	glm::vec3 force = j * n;

	c->a->linMomentum += force;
	c->b->linMomentum -= force;
	c->a->angMomentum += glm::cross(ra, force);
	c->b->angMomentum -= glm::cross(rb, force);

	c->a->velocity = c->a->linMomentum / c->a->mass;
	c->b->velocity = c->b->linMomentum / c->b->mass;

	c->a->angularVelocity = c->a->Iinv * c->a->angMomentum;
	c->b->angularVelocity = c->b->Iinv * c->b->angMomentum;
}

