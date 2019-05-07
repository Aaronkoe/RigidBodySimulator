#ifndef UTIL_FUNC_H_
#define UTIL_FUNC_H_
#include <glm/glm.hpp>
#include <iostream>

void inline PrintGlmMat3(glm::mat3 mat) {
	for (int i = 0; i < 3; i++) {
		std::cout << "[ ";
		for (int j = 0; j < 3; j++) {
			std::cout << mat[i][j] << " ";
		}
		std::cout << "]" << std::endl;
	}
}

void inline PrintGlmVec3(glm::vec3 vec) {
	std::cout << vec.x << ", " << vec.y << ", " << vec.z << std::endl;
}

#endif