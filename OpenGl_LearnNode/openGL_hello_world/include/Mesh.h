#pragma once
#include <string>
#include <types.h>
#include <vector>
#include "MyShader.h"
#include <windows.h>

#include "glm/vec3.hpp"

struct Vertex
{
	glm::vec3 position;
	glm::vec3 normal;
	glm::vec2 texCoords;
};

struct Texture
{
	unsigned int id;
	std::string type;
	aiString path;
};



class Mesh
{
public:
	///��������
	std::vector<Vertex>vertices;
	std::vector<unsigned int>indices;
	std::vector<Texture>textures;

	///����
	Mesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices, const std::vector<Texture>& textures);

	void draw(MyShader shader) const;

private:

	unsigned int VAO, VBO, EBO;

	void setupMesh();
};

