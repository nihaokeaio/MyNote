#pragma once
#include <string>
#include <types.h>
#include <vector>
#include "MyShader.h"

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
	///网格数据
	std::vector<Vertex>vertices;
	std::vector<unsigned int>indices;
	std::vector<Texture>textures;

	///函数
	Mesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices, const std::vector<Texture>& textures);

	void draw(const MyShader& shader) const;
	void drawInts(const MyShader& shader,int intsCount) const;

	void attachAttribPointer(const std::vector<glm::mat4>& nums);
private:

	unsigned int VAO, VBO, EBO;

	void setupMesh();
};

