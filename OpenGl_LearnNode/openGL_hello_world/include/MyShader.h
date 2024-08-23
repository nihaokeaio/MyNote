#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

class MyShader
{
public:
	//����Id
	unsigned int ID;
	//��������
	MyShader(const char* vertexPath, const char* fragmentPath);
	//ʹ�ã�active����ɫ��
	void use();
	static void loadTexture(const std::string& filename, unsigned int& texture);
	static void loadCubeTexture(const std::vector<std::string>& cubeFileName, unsigned int& texture);
	void setBool(const std::string& name, bool value) const;
	void setInt(const std::string& name, int value) const;
	void setFloat(const std::string& name, float value) const;
	void setVec3(const std::string& name, float x,float y,float z) const;
	void setVec3(const std::string& name, glm::vec3 vec3) const;
	void setMat4(const std::string& name, glm::mat4 mat) const;
private:
};