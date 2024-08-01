#pragma once

#include <glad/glad.h>

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

class MyShader
{
public:
	//程序Id
	unsigned int ID;
	//构造数据
	MyShader(const char* vertexPath, const char* fragmentPath);
	//使用（active）着色器
	void use();

	void setBool(const std::string& name, bool value) const;
	void setInt(const std::string& name, int value) const;
	void setFloat(const std::string& name, float value) const;
private:
};