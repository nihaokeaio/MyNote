#pragma once
#include "Vector.hpp"
#include <Eigen/src/Core/Matrix.h>


class Scene;


class Camera
{
public:
	Camera(Scene* scene);

	//两个回调函数
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
	Scene* scene_;//场景

	Vec3f cameraPos_ = { 1, 0.5f,-12.f };//摄像机位置
	Vec3f cameraFront_ = { 0.0f, 0.0f, 1.0f };//摄像机朝向
	Vec3f cameraUp_ = { 0.0f, 1.0f, 0.0f };//摄像机头顶方向

	float deltaTime_ = 0.0f; // 当前帧与上一帧的时间差
	float lastFrame_ = 0.0f; // 上一帧的时间

	float lastX_ = 400, lastY_ = 400;//鼠标上一次的位置

	float pitch_ = 0;//全局俯仰角偏移量
	float yaw_ = 0;//全局巡航叫偏移量

	bool firstMouse_ = true;

	float fov_ = 45.0f;
};

