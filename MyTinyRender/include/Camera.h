#pragma once
#include "Vector.hpp"
#include <Eigen/src/Core/Matrix.h>


class Scene;


class Camera
{
public:
	Camera(Scene* scene);

	//�����ص�����
	void processInput();
	//void mouse_callBack(double xPos, double yPos);
	void mouseCallBack(int event, int x, int y, int flags, void* param);

	Eigen::Matrix4f getViewMat();
	Eigen::Matrix4f getProjectionMat();

	void setMoveSpeed(float currentFrame);

	Vec3f getCameraPos()const;

private:
	static Eigen::Matrix4f myLookAt(const Vec3f& P, const Vec3f& T, const Vec3f& U);

private:
	Scene* scene_;//����

	Vec3f cameraPos_ = { 1, 0.5f,-12.f };//�����λ��
	Vec3f cameraFront_ = { 0.0f, 0.0f, 1.0f };//���������
	Vec3f cameraUp_ = { 0.0f, 1.0f, 0.0f };//�����ͷ������

	float deltaTime_ = 0.0f; // ��ǰ֡����һ֡��ʱ���
	float lastFrame_ = 0.0f; // ��һ֡��ʱ��

	float lastX_ = 400, lastY_ = 400;//�����һ�ε�λ��

	float pitch_ = 0;//ȫ�ָ�����ƫ����
	float yaw_ = 0;//ȫ��Ѳ����ƫ����

	bool firstMouse_ = true;

	float fov_ = 45.0f;
};

