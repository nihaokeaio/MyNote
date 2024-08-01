#pragma once

#include <glad/glad.h>

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

class MyShader
{
public:
	//����Id
	unsigned int ID;
	//��������
	MyShader(const char* vertexPath, const char* fragmentPath);
	//ʹ�ã�active����ɫ��
	void use();

	void setBool(const std::string& name, bool value) const;
	void setInt(const std::string& name, int value) const;
	void setFloat(const std::string& name, float value) const;
private:
};