#include "RigidBodySimulator.h"

void RigidBodySimulator::UpdateLoop(float dt)
{
	//for (int i = 0; i < 10; i++) {
		AdvanceTime(dt);
		if (AreAnyCollisions()) {
			AdvanceTime(-dt/10);
			AdvanceToCollisionDt(dt/10);
			HandleCollisions();
		}
	//}
}

void RigidBodySimulator::AdvanceTime(float dt)
{
	for (int i = 0; i < rigidBodies.size(); ++i) {
		rigidBodies[i]->Update(dt);
	}
}

void RigidBodySimulator::DrawBodies(Shader& shader, unsigned int vao)
{
	for (int i = 0; i < rigidBodies.size(); ++i) {
		rigidBodies[i]->Draw(shader, vao);
	}
	for (int i = 0; i < walls.size(); ++i) {
		walls[i]->Draw(shader, vao);
	}
}

bool RigidBodySimulator::AreAnyCollisions()
{
	collisions.clear();
	for (int i = 0; i < rigidBodies.size(); ++i) {
		for (int j = 0; j < rigidBodies.size(); ++j) {
			int vertexColliding = rigidBodies[i]->IsMyVertexInside(rigidBodies[j]);
			if (i != j && vertexColliding != -1) {
				collisions.push_back(Collision(rigidBodies[i], rigidBodies[j], vertexColliding));
			}
		}
		for (int j = 0; j < walls.size(); j++) {
			int vertexColliding = rigidBodies[i]->IsMyVertexInside(walls[j]);
			if (i != j && vertexColliding != -1) {
				collisions.push_back(Collision(rigidBodies[i], walls[j], vertexColliding));
			}
		}
	}
	return collisions.size() != 0;
}

float RigidBodySimulator::AdvanceToCollisionDt(float dt)
{
	float dtChunkSize = dt / collisionSteps;
	for (int i = 1; i < collisionSteps + 1; ++i) {
		for (int j = 0; j < collisions.size(); ++j) {
			if (collisions[j].a->IsMyVertexInside(collisions[j].b)) {
				AdvanceTime(-dtChunkSize);
				return dtChunkSize * (i - 1);
			}
		}
		AdvanceTime(dtChunkSize);
	}
	return -1;
}

void RigidBodySimulator::HandleCollisions()
{
	constexpr float epsilon = .3;
	std::cout << "handling a collision" << std::endl;
	for (int i = 0; i < collisions.size(); ++i) {
		Collision& currCollision = collisions[i];
		RigidBody2D* vertexBody = currCollision.a;
		RigidBody2D* faceBody = currCollision.b;
		float omegaA1 = vertexBody->angularVelocity;
		float omegaB1 = faceBody->angularVelocity;
		glm::vec2 va1 = vertexBody->linearVelocity;
		glm::vec2 vb1 = faceBody->linearVelocity;
		glm::vec2 collisionPoint = vertexBody->PointBodyToWorld(vertexBody->vertices[currCollision.vertexThatIsColliding]);
		glm::vec2 rap = -vertexBody->position + collisionPoint;
		glm::vec2 rbp = -faceBody->position + collisionPoint;
		glm::vec2 n = faceBody->GetNormalOfPoint(collisionPoint);
		glm::vec2 vap1 = vertexBody->linearVelocity + glm::vec2(-omegaA1 * rap.x, omegaA1 * rap.y);
		glm::vec2 vbp1 = faceBody->linearVelocity + glm::vec2(-omegaB1 * rbp.x, omegaB1 * rbp.y);
		glm::vec2 vab1 = vap1 - vbp1;
		float j;
		float numerator = -(1 + epsilon) * glm::dot(vab1, n);
		float denom1 = 1 / vertexBody->mass;
		float denom2 = 1 / faceBody->mass;
		float denom3 = glm::dot(glm::vec2(rap.x * n.y, -n.x * rap.y), glm::vec2(rap.x * n.y, -n.x * rap.y)) / vertexBody->momentOfInertia;
		float denom4 = glm::dot(glm::cross(glm::vec3(rbp.x, rbp.y, 0), glm::vec3(n.x, n.y, 0)), glm::cross(glm::vec3(rbp.x, rbp.y, 0), glm::vec3(n.x, n.y, 0))) / faceBody->momentOfInertia;
		j = abs(numerator / (denom1 + denom2 + denom3 + denom4));
		glm::vec2 va2 = va1 + j * n / vertexBody->mass;
		glm::vec2 vb2 = vb1 - j * n / faceBody->mass;
		float omegaA2 = omegaA1 + (rap.x * (j * n).y - rap.y * (j * n).x) / vertexBody->momentOfInertia;
		float omegaB2 = omegaB1 - (rbp.x * (j * n).y - rbp.y * (j * n).x) / faceBody->momentOfInertia;
		vertexBody->angularVelocity = omegaA2;
		vertexBody->linearVelocity = va2;
		faceBody->linearVelocity = vb2;
		faceBody->angularVelocity = omegaB2;
		std::cout << j << std::endl;
	}
}

void RigidBodySimulator::ApplyAcceleration(glm::vec2 accel)
{
	for (int i = 0; i < rigidBodies.size(); ++i) {
		rigidBodies[i]->ApplyForce(accel, glm::vec2(0, 0));
	}
}
