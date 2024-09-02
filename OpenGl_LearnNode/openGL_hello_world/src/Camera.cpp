#include "Camera.h"
#include <glm/gtc/matrix_transform.hpp>
//#include <functional>
#include <iostream>

Camera::Camera(GLFWwindow* w, float width=400.0f, float height=400.0f)
{
    window = w;
    windowH = height;
    windowW = width;
}

void Camera::processInput()
{
    float cameraSpeed = 2.5f * deltaTime; // adjust accordingly
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
    { 
        /*glm::vec3 tmp = cameraFront;
        tmp.y = 0;
        tmp = glm::normalize(tmp);
        cameraPos -= cameraSpeed * tmp;*/
        cameraPos += cameraSpeed * cameraFront;
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
    {
        /*glm::vec3 tmp = cameraFront;
        tmp.y = 0;
        tmp = glm::normalize(tmp);
        cameraPos -= cameraSpeed * tmp;*/
        cameraPos -= cameraSpeed * cameraFront;
    }

    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
    {
        cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
    {
        cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    }
    ///原神水下移动玩法
    if(glfwGetKey(window,GLFW_KEY_SPACE)==GLFW_PRESS)
    {
        cameraPos += cameraUp * cameraSpeed;
    }

    if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
    {
        cameraPos -= cameraUp * cameraSpeed;
    }

}

void Camera::mouse_callBack(double xPos, double yPos)
{
    if (firstMouse) // 这个bool变量初始时是设定为true的
    {
        lastX = xPos;
        lastY = yPos;
        firstMouse = false;
        return;
    }
    float xOffset = xPos - lastX;
    float yOffset = lastY - yPos;//由于这里左下角为（0，0），与传统的坐标系统不同，因此y轴需要取反
    lastX = xPos;
    lastY = yPos;
    float sensitivity = 0.05f;
    xOffset *= sensitivity;
    yOffset *= sensitivity;

    pitch += yOffset;
    yaw += xOffset;

    //添加一些限制，防止视角出现奇怪的现象
    if (pitch >= 89.0f)
    {
        pitch = 89.0f;
    }
    if (pitch <= -89.0f)
    {
        pitch = -89.0f;
    }
    glm::vec3 front;
    front.x = cos(glm::radians(pitch)) * sin(glm::radians(yaw));
    front.y =  sin(glm::radians(pitch));
    front.z = -cos(glm::radians(pitch)) * cos(glm::radians(yaw));
    cameraFront = glm::normalize(front);
}

void Camera::scroll_callBack(double xOffset, double yOffset)
{
    if (fov >= 1.0f && fov <= 60.0f)
        fov -= yOffset;
    if (fov <= 1.0f)
        fov = 1.0f;
    if (fov >= 60.0f)
        fov = 60.0f;
}

glm::mat4 Camera::getViewMat()
{
    /*return glm::lookAt(
        cameraPos,
        cameraPos + cameraFront,
        cameraUp
    );*/
    return myLookAt(cameraPos, cameraPos + cameraFront, cameraUp);
}

glm::mat4 Camera::getProjectionMat()
{
    float aspect = windowW / windowH;  
    return glm::perspective(glm::radians(fov), aspect, 0.1f, 100.0f);//projection

}

void Camera::setMoveSpeed(float currentFrame)
{
    deltaTime = currentFrame - lastFrame;
    lastFrame = currentFrame;
    std::cout << "FPS = " << static_cast<float>(1.0f / deltaTime) << " \r";
    std::cout.flush();
}

glm::mat4 Camera::myLookAt(glm::vec3 P, glm::vec3 T, glm::vec3 U)
{
    glm::vec3 zAxis = glm::normalize(P-T);
    glm::vec3 xAxis = glm::normalize(glm::cross(glm::normalize(U), zAxis));
    glm::vec3 yAxis = glm::normalize(glm::cross(zAxis, xAxis));

    glm::mat4 transMat = glm::mat4(1.0f);
    transMat = glm::translate(transMat, -P);
    /*transMat[3][0] = -P.x;
    transMat[3][1] = -P.y;
    transMat[3][2] = -P.z;*/

    glm::mat4 rotateMat = glm::mat4(1.0f);
    rotateMat[0][0] = xAxis.x;
    rotateMat[1][0] = xAxis.y;
    rotateMat[2][0] = xAxis.z;

    rotateMat[0][1] = yAxis.x;
    rotateMat[1][1] = yAxis.y;
    rotateMat[2][1] = yAxis.z;

    rotateMat[0][2] = zAxis.x;
    rotateMat[1][2] = zAxis.y;
    rotateMat[2][2] = zAxis.z;

    return rotateMat * transMat;
}

glm::vec3 Camera::getCameraPos() const
{
    return cameraPos;
}

void Camera::showFPS()
{
    
}
