#pragma once
#ifndef RIGID_BODY_H_
#define RIGID_BODY_H_
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GLFW/glfw3.h>
#include <learnopengl/shader_m.h>
#include "UtilFunc.h"


class RigidBody2D {
public:
	RigidBody2D(float m, float w, float h);
	void Draw(Shader& shader, unsigned int vao);
	void ApplyForce(glm::vec2 force, glm::vec2 arm);
	void Update(float dt);
	void FloorCollision();
	glm::vec2 Rotate(glm::vec2);
	glm::vec2 GetVelocityOfPoint(glm::vec2);
	glm::vec2 PointBodyToWorld(glm::vec2);
	glm::vec2 PointWorldToBody(glm::vec2);
	bool IsPointInside(glm::vec2 worldSpacePoint);
	bool AreBodiesColliding(RigidBody2D* otherBody);
	void HandleCollision(RigidBody2D* otherBody);
	glm::vec2 GetNormalOfPoint(glm::vec2 worldSpacePoint);

	glm::vec2 position;
	glm::vec2 linearVelocity;
	glm::vec2 force;

	float angle;
	float angularVelocity;
	float torque;

	float width;
	float height;
	float mass;
	float momentOfInertia;

	glm::vec2 vertices[4];
};





struct Contact;
class CarnegieRigidBody {
public:
	CarnegieRigidBody(float mass, float width, float height, float depth);
	void ApplyImpulse(glm::vec3 force, glm::vec3 r);
	void Update(float dt);
	void PrintState();
	void PrintForceAndTorque();
	glm::vec3 pt_velocity(glm::vec3 p);
	bool colliding(Contact* c);
	void collision(Contact* c, double epsilon);
	void find_all_collisions(Contact contacts[], int ncontacts);
	glm::vec3 WorldToBodySpace(glm::vec3 p);
	glm::vec3 BodyToWorldSpace(glm::vec3 p);
	void FloorCollision();

	//for a cube
	float width, height, depth;

	float mass;
	glm::mat3 Ibody, IbodyInv;

	glm::vec3 position;    // x(t)
	glm::mat3 rotation;    // R(t)
	glm::quat orientation;  // q(t)
	glm::vec3 linMomentum; // P(t)
	glm::vec3 angMomentum; // L(t)
	
	glm::mat3 I;
	glm::mat3 Iinv;        // changes whenever the body moves?
	glm::vec3 velocity;
	glm::vec3 angularVelocity;
	
	glm::vec3 force;
	glm::vec3 torque;

	glm::vec3 vertices[8];
};
#endif