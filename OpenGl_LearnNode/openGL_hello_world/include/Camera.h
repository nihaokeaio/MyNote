#pragma once
#include <glm/glm.hpp>
#include <GLFW/glfw3.h>

class Camera
{
public:
	Camera(GLFWwindow* w, float width, float height);

	//三个回调函数
	void processInput();
	void mouse_callBack(double xPos, double yPos);
	void scroll_callBack(double xOffset, double yOffset);

	glm::mat4 getViewMat();
	glm::mat4 getProjectionMat();

	void setMoveSpeed(float currentFrame);
	glm::mat4 myLookAt(glm::vec3 P, glm::vec3 T, glm::vec3 U);

	glm::vec3 getCameraPos()const;
private:
	GLFWwindow* window;//场景
	float windowH, windowW;

	glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 3.0f);//摄像机位置
	glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);//摄像机朝向
	glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);//摄像机头顶方向

	float deltaTime = 0.0f; // 当前帧与上一帧的时间差
	float lastFrame = 0.0f; // 上一帧的时间

	float lastX = 400, lastY = 400;//鼠标上一次的位置

	float pitch = 0;//全局俯仰角偏移量
	float yaw = 0;//全局巡航叫偏移量

	bool firstMouse = true;

	float fov = 45.0f;
};