#include "MyScene.h"
#include "Camera.h"

#include <string>
#include <iostream>

#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>

#include "imgui.h"
#include "backends/imgui_impl_opengl3.h"
#include "backends/imgui_impl_glfw.h"
#include "misc/cpp/imgui_stdlib.h"



class MyScene::PImpl
{
public:
    void init(const std::string& windowTitle = "window");

    //初始化上下文
    void initContext();

    //注册回调函数
    void registerCallBackFun();

    //设置输入模式
    void setInputMode(int mode = GLFW_CURSOR, int value = GLFW_CURSOR_HIDDEN);

    //添加摄像机
    void addCamera();
private:
    bool initGlfwContext();
    bool initImguiContext();

    void mouseCallback(double xPos, double yPos);
    void scrollCallBack(double xOffset, double yOffset);
    void processHandler(GLFWwindow* window);

    // 鼠标回调函数（静态成员函数）
    static void framebufferSizeCallback(GLFWwindow* window, int width, int height);
    static void mouseCallback(GLFWwindow* window, double xPos, double yPos);
    static void scrollCallBack(GLFWwindow* window, double xOffset, double yOffset);
    static void processInput(GLFWwindow* window);

public:
    std::string windowTitle_;
    int scrWidth_ = 800;
    int scrHeight_ = 800;
	GLFWwindow* window_;
    std::unique_ptr<Camera> camera_;
};

MyScene::MyScene()
{
	impl_.reset(new PImpl);
}

MyScene::~MyScene() = default;

void MyScene::PImpl::init(const std::string& windowTitle)
{
    windowTitle_ = windowTitle;
    initContext();
    registerCallBackFun();
    setInputMode();
}

void MyScene::PImpl::initContext()
{
    initGlfwContext();
    initImguiContext();
}

void MyScene::PImpl::registerCallBackFun()
{
    //设置三个回调函数
     // 将当前实例指针与 GLFWwindow 关联
    glfwSetWindowUserPointer(window_, this);

    // 设置回调函数
    //glfwSetCursorPosCallback(window_, mouseCallback);

    glfwSetFramebufferSizeCallback(window_, framebufferSizeCallback);
    glfwSetCursorPosCallback(window_, mouseCallback);
    glfwSetScrollCallback(window_, scrollCallBack);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return;
    }
}

void MyScene::PImpl::setInputMode(int mode, int value)
{
    //聚焦光标
    glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
}


void MyScene::PImpl::addCamera()
{
    camera_.reset(new Camera(window_, scrWidth_, scrHeight_));
}

bool MyScene::PImpl::initGlfwContext()
{
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    //创建一个面板
    window_ = glfwCreateWindow(scrWidth_, scrHeight_, windowTitle_.c_str(), nullptr, nullptr);
    if (!window_)
    {
        std::cout << "Failed to create window" << std::endl;
        glfwTerminate();
        return false;
    }

    //设置关联环境
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1); // 启用垂直同步

}

bool MyScene::PImpl::initImguiContext()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;

    // 设置 ImGui 样式
    ImGui::StyleColorsDark();

    // 初始化 ImGui 的 GLFW 和 OpenGL3 后端
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");
    return false;
}

void MyScene::PImpl::mouseCallback(double xPos, double yPos)
{
    camera_->mouse_callBack(xPos, yPos);
}

void MyScene::PImpl::scrollCallBack(double xOffset, double yOffset)
{
    camera_->scroll_callBack(xOffset, yOffset);
}

void MyScene::PImpl::processHandler(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    camera_->processInput();
}

void MyScene::PImpl::framebufferSizeCallback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void MyScene::PImpl::mouseCallback(GLFWwindow* window, double xPos, double yPos)
{
    MyScene::PImpl* instance = static_cast<MyScene::PImpl*>(glfwGetWindowUserPointer(window));
    if (instance) 
    {
        instance->mouseCallback(xPos, yPos);
    }
}

void MyScene::PImpl::scrollCallBack(GLFWwindow* window, double xOffset, double yOffset)
{
    MyScene::PImpl* instance = static_cast<MyScene::PImpl*>(glfwGetWindowUserPointer(window));
    if (instance)
    {
        instance->scrollCallBack(xOffset, yOffset);
    }
}

void MyScene::PImpl::processInput(GLFWwindow* window)
{
    MyScene::PImpl* instance = static_cast<MyScene::PImpl*>(glfwGetWindowUserPointer(window));
    if (instance)
    {       
        instance->processHandler(window);
    }
}
