#include "Camera.h"

#include <opencv2/highgui.hpp>

#include "global.hpp"
#include "Scene.h"

Camera::Camera(Scene* scene)
{
    scene_ = scene;
}

void Camera::processInput()
{
    float cameraSpeed = 2.5f * deltaTime_; // adjust accordingly

    int key = 0;
    if (key != 27)
    {
        key = cv::waitKey(10);

        switch (key)
        {
        case 27:
            return;

        case 'A':
        {
            cameraPos_ -= cameraFront_.cross(cameraUp_).normalize() * cameraSpeed;
            break;
        }

        case 'S':
        {
            cameraPos_ -= cameraSpeed * cameraFront_;
            break;
        }

        case 'D':
        {
            cameraPos_ += cameraFront_.cross(cameraUp_).normalize() * cameraSpeed;
            break;
        }

        case 'W':
        {
            cameraPos_ += cameraSpeed * cameraFront_;
            break;
        }

        default:;
        }
    }
    
    /*
     ///原神水下移动玩法
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
    {
        cameraPos += cameraUp * cameraSpeed;
    }

    if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
    {
        cameraPos -= cameraUp * cameraSpeed;
    }
    */
}

void Camera::mouseCallBack(int event, int x, int y, int flags, void* param)
{
    switch (event)
    {
	    case cv::EVENT_MOUSEMOVE:
	    {
	        if (firstMouse_) // 这个bool变量初始时是设定为true的
	        {
	            lastX_ = x;
	            lastY_ = y;
	            firstMouse_ = false;
	            return;
	        }
	        float xOffset = x - lastX_;
	        float yOffset = lastY_ - y;//由于这里左下角为（0，0），与传统的坐标系统不同，因此y轴需要取反
	        lastX_ = x;
	        lastY_ = y;
	        constexpr float sensitivity = 0.05f;
	        xOffset *= sensitivity;
	        yOffset *= sensitivity;

	        pitch_ += yOffset;
	        yaw_ += xOffset;

	        //添加一些限制，防止视角出现奇怪的现象
	        if (pitch_ >= 89.0f)
	        {
	            pitch_ = 89.0f;
	        }
	        if (pitch_ <= -89.0f)
	        {
	            pitch_ = -89.0f;
	        }
	        Vec3f front;

	        auto getRadians = [](float angle)
	            {
	                return angle / 180.f * M_PI;
	            };
	        front.x = cos(getRadians(pitch_)) * sin(getRadians(yaw_));
	        front.y = sin(getRadians(pitch_));
	        front.z = -cos(getRadians(pitch_)) * cos(getRadians(yaw_));
	        cameraFront_ = front.normalize();
	        break;
	    }
	    case cv::EVENT_MOUSEWHEEL:
	    {
		    const auto& value = cv::getMouseWheelDelta(flags);

            if (fov_ >= 1.0f && fov_ <= 60.0f)
                fov_ -= value * 1.0f;
            if (fov_ <= 1.0f)
                fov_ = 1.0f;
            if (fov_ >= 60.0f)
                fov_ = 60.0f;
	    	break;
	    }
    }
}

Eigen::Matrix4f Camera::getViewMat()
{
    return myLookAt(cameraPos_, cameraPos_ + cameraFront_, cameraUp_);
}

Eigen::Matrix4f Camera::getProjectionMat()
{
    float aspect = scene_->width_ / scene_->height_;

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    float zNear = -scene_->near_;
    float zFar = -scene_->far_;
    projection << zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, zNear + zFar, -zNear * zFar,
        0, 0, 1, 0;
    Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
    float H = 2 * -zNear * tan(fov_ / 180 * M_PI / 2);
    float W = aspect * H;

    Eigen::Matrix4f ortho = Eigen::Matrix4f::Identity();

    ortho << 2 / W, 0, 0, 0,
        0, 2 / H, 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;

    projection = scale * ortho * projection;

    return projection;
}

void Camera::setMoveSpeed(double currentFrame)
{
    deltaTime_ = 1;// currentFrame - lastFrame_;
    lastFrame_ = 0; //currentFrame;
}

Vec3f Camera::getCameraPos() const
{
    return cameraPos_;
}


Eigen::Matrix4f Camera::myLookAt(const Vec3f& P, const Vec3f& T, const Vec3f& U)
{
	const Vec3f zAxis = (P - T).normalize();
	const Vec3f xAxis = U.cross(zAxis).normalize();
    Vec3f yAxis = zAxis.cross(xAxis);

    Eigen::Matrix4f matRotate = Eigen::Matrix4f::Identity();

    matRotate << xAxis.x, yAxis.x, zAxis.x, -P.x,
        xAxis.y, yAxis.y, zAxis.y, -P.y,
        xAxis.z, yAxis.z, zAxis.z, -P.z,
        0, 0, 0, 1;

    return matRotate;
}
