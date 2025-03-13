﻿#pragma once

#include <glm/glm.hpp>

#include <string>
#include <vector>

class MyShader
{
public:
	//程序Id
	unsigned int ID;
	//构造数据
	MyShader(const char* vertexPath, const char* fragmentPath);
	bool attachShader(const char* shaderPath, int GLuint);
	//使用（active）着色器
	void use();
	static void loadTexture(const std::string& filename, unsigned int& texture);
	static void loadCubeTexture(const std::vector<std::string>& cubeFileName, unsigned int& texture);
	void setBool(const std::string& name, bool value) const;
	void setInt(const std::string& name, int value) const;
	void setFloat(const std::string& name, float value) const;
	void setVec2(const std::string& name, float x,float y) const;
	void setVec2(const std::string& name, glm::vec2 vec2) const;
	void setVec3(const std::string& name, float x,float y,float z) const;
	void setVec3(const std::string& name, glm::vec3 vec3) const;
	void setMat4(const std::string& name, glm::mat4 mat) const;
private:
};