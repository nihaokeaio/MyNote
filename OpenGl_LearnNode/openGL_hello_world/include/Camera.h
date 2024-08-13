#pragma once
#include <glm/glm.hpp>
#include <GLFW/glfw3.h>

class Camera
{
public:
	Camera(GLFWwindow* w, float width, float height);

	//�����ص�����
	void processInput();
	void mouse_callBack(double xPos, double yPos);
	void scroll_callBack(double xOffset, double yOffset);

	glm::mat4 getViewMat();
	glm::mat4 getProjectionMat();

	void setMoveSpeed(float currentFrame);
	glm::mat4 myLookAt(glm::vec3 P, glm::vec3 T, glm::vec3 U);

	glm::vec3 getCameraPos()const;
private:
	GLFWwindow* window;//����
	float windowH, windowW;

	glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 3.0f);//�����λ��
	glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);//���������
	glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);//�����ͷ������

	float deltaTime = 0.0f; // ��ǰ֡����һ֡��ʱ���
	float lastFrame = 0.0f; // ��һ֡��ʱ��

	float lastX = 400, lastY = 400;//�����һ�ε�λ��

	float pitch = 0;//ȫ�ָ�����ƫ����
	float yaw = 0;//ȫ��Ѳ����ƫ����

	bool firstMouse = true;

	float fov = 45.0f;
};