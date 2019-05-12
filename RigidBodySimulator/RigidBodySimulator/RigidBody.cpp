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
	this->orientation = glm::quat(1, 0, 0, 0);
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

	this->vertices[0] = glm::vec3(width / 2, height / 2, depth / 2);
	this->vertices[1] = glm::vec3(width / 2, height / 2, -depth / 2);
	this->vertices[2] = glm::vec3(width / 2, -height / 2, depth / 2);
	this->vertices[3] = glm::vec3(width / 2, -height / 2, -depth / 2);
	this->vertices[4] = glm::vec3(-width / 2, height / 2, depth / 2);
	this->vertices[5] = glm::vec3(-width / 2, height / 2, -depth / 2);
	this->vertices[6] = glm::vec3(-width / 2, -height / 2, depth / 2);
	this->vertices[7] = glm::vec3(-width / 2, -height / 2, -depth / 2);
}

// force and r are in world space, r needs to change to body space
void CarnegieRigidBody::ApplyImpulse(glm::vec3 force, glm::vec3 r) {
	glm::vec3 rBody = r - position;
	this->force += force;
	this->torque += glm::cross(rBody, force);
}

// here is where gravity is applied
void CarnegieRigidBody::Update(float dt) {
	//this->force += glm::vec3(0, -.00098, 0);
	this->velocity = linMomentum / mass;
	this->I = this->rotation * this->Ibody * glm::transpose(this->rotation);
	this->Iinv = glm::inverse(I);
	this->angularVelocity = Iinv * angMomentum;
	glm::vec3 v1 = angularVelocity;
	glm::vec3 v2 = glm::vec3(orientation[0], orientation[1], orientation[2]);
	float s2 = orientation[3];
	glm::vec4 quatProd = glm::vec4(s2 * v1 + glm::cross(v1, v2), glm::dot(v1, v2));

	this->position += velocity * dt;
	this->orientation += glm::quat(.5f * (quatProd)* dt);
	this->orientation = glm::normalize(orientation);
	this->linMomentum += force * dt;
	this->linMomentum *= .9995f;
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
void CarnegieRigidBody::collision(Contact* c, double epsilon)
{
	glm::vec3 padot = c->a->pt_velocity(c->p);
	glm::vec3 pbdot = glm::vec3(0, 0, 0);
	if (c->b != NULL) {
		pbdot = c->b->pt_velocity(c->p);
	}
	glm::vec3 n = c->n,
		ra = c->p - c->a->position
		;// rb = c->p - c->b->position;
	double vrel = glm::dot(n, (padot - pbdot)),
		numerator = -(1 + epsilon) * vrel;
	double term1 = 1 / c->a->mass,
		term2 = 0,//1 / c->b->mass,
		term3 = glm::dot(n, ((c->a->Iinv * glm::cross((glm::cross(ra, n)), ra)))),
		term4 = 0;//glm::dot(n, ((c->b->Iinv * glm::cross((glm::cross(rb, n)), rb))));
	float j = numerator / (term1 + term2 + term3 + term4);
	glm::vec3 force = j * n;

	if (c->b != NULL) {
		c->a->linMomentum += force;
		c->b->linMomentum -= force;
		c->a->angMomentum += glm::cross(ra, force);
		//c->b->angMomentum -= glm::cross(rb, force);

		c->a->velocity = c->a->linMomentum / c->a->mass;
		c->b->velocity = c->b->linMomentum / c->b->mass;

		c->a->angularVelocity = c->a->Iinv * c->a->angMomentum;
		c->b->angularVelocity = c->b->Iinv * c->b->angMomentum;
	}
	else {
		c->a->linMomentum += force * 2.0f;
		c->a->angMomentum += glm::cross(ra, force * 2.0f);

		c->a->velocity = c->a->linMomentum / c->a->mass;

		c->a->angularVelocity = c->a->Iinv * c->a->angMomentum;

	}
}
void CarnegieRigidBody::find_all_collisions(Contact contacts[], int ncontacts)
{
	bool had_collision;
	double epsilon = .5;

	do {
		had_collision = FALSE;
		for (int i = 0; i < ncontacts; i++) {
			if (colliding(&contacts[i])) {
				collision(&contacts[i], epsilon);
				had_collision = TRUE;
				//ode_discontinuos();
			}
		}
	} while (had_collision == TRUE);
}

glm::vec3 CarnegieRigidBody::WorldToBodySpace(glm::vec3 p)
{
	glm::mat3 rotation = glm::toMat3(orientation);
	rotation = glm::inverse(rotation);
	return rotation * (p - position);
}

glm::vec3 CarnegieRigidBody::BodyToWorldSpace(glm::vec3 p)
{
	glm::mat3 rotation = glm::toMat3(orientation);
	return rotation * p + position;
}

void CarnegieRigidBody::FloorCollision()
{
	for (int i = 0; i < 8; ++i) {
		if (BodyToWorldSpace(vertices[i]).y < 0) {
			glm::vec3 pointVel = pt_velocity(BodyToWorldSpace(vertices[i]));
			Contact* contact = new Contact();
			contact->a = this;
			contact->b = NULL;
			contact->p = BodyToWorldSpace(vertices[i]);
			contact->n = glm::vec3(0, 1, 0);
			contact->vertexFace = true;
			collision(contact, .7);
			delete contact;
		}
	}
}

RigidBody2D::RigidBody2D(float m, float w, float h)
{
	position = glm::vec2(0, 0);
	linearVelocity = glm::vec2(0, 0);
	force = glm::vec2(0, 0);
	angle = 0;
	angularVelocity = 0;
	torque = 0;
	width = w;
	height = h;
	mass = m;
	momentOfInertia = mass * (width * width + height * height) / 12;

	vertices[0] = glm::vec2(width / 2, height / 2);
	vertices[1] = glm::vec2(width / 2, -height / 2);
	vertices[2] = glm::vec2(-width / 2, height / 2);
	vertices[3] = glm::vec2(-width / 2, -height / 2);
}

void RigidBody2D::Draw(Shader& shader, unsigned int vao)
{
	glm::mat4 model = glm::mat4(1.0);
	model = glm::translate(model, glm::vec3(position.x, position.y, 0));
	model = glm::rotate(model, angle, glm::vec3(0, 0, 1));
	model = glm::scale(model, glm::vec3(width, height, 1));
	shader.setMat4("model", model);
	glBindVertexArray(vao);
	glDrawArrays(GL_TRIANGLES, 0, 36);
}

void RigidBody2D::ApplyForce(glm::vec2 force, glm::vec2 arm)
{
	this->force += force;
	this->torque += arm.x * force.y - arm.y * force.x;
}

void RigidBody2D::Update(float dt)
{
	glm::vec2 accel = force / mass;
	linearVelocity += accel * dt;
	position += linearVelocity * dt;

	float angularAcceleration = torque / momentOfInertia;
	angularVelocity += angularAcceleration * dt;
	angle += angularVelocity * dt;
	
	force = glm::vec2(0, 0);
	torque = 0;
}

void RigidBody2D::FloorCollision()
{
	for (int i = 0; i < 4; i++) {
		if (PointBodyToWorld(vertices[i]).y < 0 && GetVelocityOfPoint(vertices[i]).y < 0) {
			glm::vec2 worldSpacePointOfContact = PointBodyToWorld(vertices[i]);
			glm::vec2 ra = worldSpacePointOfContact - position;
			glm::vec2 normal = glm::vec2(0, 1);
			glm::vec2 padot = GetVelocityOfPoint(worldSpacePointOfContact);
			double vrel = glm::dot(normal, padot),
				numerator = -(1 + .03) * vrel;
			float j = numerator;
			glm::vec2 force = j * normal;
			linearVelocity += force;
			angularVelocity += ra.x * force.y - ra.y * force.x;
		}
	}
}

glm::vec2 RigidBody2D::Rotate(glm::vec2 p)
{
	glm::mat2 rotation = glm::mat2();
	rotation[0][0] = cos(angle);
	rotation[1][0] = -sin(angle);
	rotation[0][1] = sin(angle);
	rotation[1][1] = cos(angle);
	return rotation * p;
}

glm::vec2 RigidBody2D::GetVelocityOfPoint(glm::vec2 bodySpacePoint)
{
	return linearVelocity + angularVelocity * (bodySpacePoint - position);
}

// double check these rotation matrices. row major or column major?
glm::vec2 RigidBody2D::PointBodyToWorld(glm::vec2 p)
{
	glm::mat2 rotation = glm::mat2();
	rotation[0][0] = cos(angle);
	rotation[1][0] = -sin(angle);
	rotation[0][1] = sin(angle);
	rotation[1][1] = cos(angle);
	return rotation * p + position;
}

glm::vec2 RigidBody2D::PointWorldToBody(glm::vec2 p)
{
	glm::mat2 rot = glm::mat2();
	rot[0][0] = cos(angle);
	rot[1][0] = -sin(angle);
	rot[0][1] = sin(angle);
	rot[1][1] = cos(angle);
	rot = glm::inverse(rot);
	return rot * (p - position);
}

// need to test this still
bool RigidBody2D::IsPointInside(glm::vec2 worldSpacePoint)
{
	glm::vec2 bodySpacePoint = PointWorldToBody(worldSpacePoint);
	if (bodySpacePoint.x <= width / 2 && bodySpacePoint.x >= -width / 2 && bodySpacePoint.y <= height / 2 && bodySpacePoint.y >= -height / 2) {
		return true;
	}
	return false;
}

bool RigidBody2D::AreBodiesColliding(RigidBody2D* otherBody)
{
	for (int i = 0; i < 4; ++i) {
		if (otherBody->IsPointInside(PointBodyToWorld(vertices[i]))) {
			std::cout << "hi";
			return true;
		}
	}
	for (int i = 0; i < 4; ++i) {
		if (IsPointInside(otherBody->PointBodyToWorld(otherBody->vertices[i]))) {
			std::cout << "hi";
			return true;
		}
	}
	return false;
}

void RigidBody2D::HandleCollision(RigidBody2D* otherBody)
{
	bool thisIsVertex = false;
	glm::vec2 worldSpacePointOfContact;
	glm::vec2 normal;
	for (int i = 0; i < 4; ++i) {
		if (otherBody->IsPointInside(PointBodyToWorld(vertices[i]))) {
			worldSpacePointOfContact = PointBodyToWorld(vertices[i]);
			normal = GetNormalOfPoint(worldSpacePointOfContact);
			thisIsVertex = true;
			break;
		}
	}
	for (int i = 0; i < 4; ++i) {
		if (IsPointInside(otherBody->PointBodyToWorld(otherBody->vertices[i]))) {
			worldSpacePointOfContact = otherBody->PointBodyToWorld(vertices[i]);
			normal = otherBody->GetNormalOfPoint(worldSpacePointOfContact);
			thisIsVertex = false;
			break;
		}
	}
	this->ApplyForce(glm::vec2(.01, 0), glm::vec2(0, 0));
	glm::vec2 padot = GetVelocityOfPoint(worldSpacePointOfContact);
	glm::vec2 pbdot = otherBody->GetVelocityOfPoint(worldSpacePointOfContact);
	glm::vec2 ra = worldSpacePointOfContact - position;
	glm::vec2 rb = worldSpacePointOfContact - otherBody->position;
	double vrel = glm::dot(normal, (padot - pbdot)),
		numerator = -(1 + .5) * vrel;
	double term1 = 1 / mass,
		term2 = 1 / otherBody->mass,
		term3 = 0,//glm::dot(normal, ((c->a->Iinv * glm::cross((glm::cross(ra, normal)), ra)))),
		term4 = 0;//glm::dot(n, ((c->b->Iinv * glm::cross((glm::cross(rb, n)), rb))));
	float j = numerator / (term1 + term2 + term3 + term4);
	glm::vec2 force = j * normal;

	//c->a->linMomentum += force;
	//c->b->linMomentum -= force;
	//c->a->angMomentum += glm::cross(ra, force);
	//c->b->angMomentum -= glm::cross(rb, force);

	linearVelocity += force;
	otherBody->linearVelocity -= force;

	angularVelocity += ra.x * force.y - ra.y * force.x;
	otherBody->angularVelocity -= rb.x * force.y - rb.y * force.x;
}

glm::vec2 RigidBody2D::GetNormalOfPoint(glm::vec2 worldSpacePoint)
{
	glm::vec2 bodySpacePoint = PointWorldToBody(worldSpacePoint);
    // need to do this better
	int closestVert = 0;
	float dist = glm::length(bodySpacePoint - vertices[0]);
	for (int i = 1; i < 4; i++) {
		float newDist = glm::length(bodySpacePoint - vertices[i]);
		if (newDist < dist) {
			closestVert = i;
			dist = newDist;
		}
	}
	int otherVertex;
	int otherVertex1 = (closestVert - 1 + 4) % 4;
	int otherVertex2 = (closestVert + 1) % 4;
	if (glm::dot(vertices[otherVertex1] - vertices[closestVert], bodySpacePoint) < glm::dot(vertices[otherVertex2] - vertices[closestVert], bodySpacePoint)) {
		otherVertex = otherVertex2;
	}
	else {
		otherVertex = otherVertex1;
	}
	return glm::normalize(worldSpacePoint - glm::proj(worldSpacePoint, vertices[otherVertex] - vertices[closestVert]));
}
