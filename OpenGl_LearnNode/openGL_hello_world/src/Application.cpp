﻿#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <memory>
#include "stb_image.h"

#include <MyShader.h>
#include <Camera.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "Model.h"

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);
void mouse_callBack(GLFWwindow* window, double xPos, double yPos);
void scroll_callBack(GLFWwindow* window, double xOffset, double yOffset);
// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;
std::shared_ptr<Camera> camera;

//glm::vec3 lightPos(1.2f, 0.5f, 2.0f);
glm::vec3 lightColor(1.0f, 1.0f, 1.0f);
glm::vec3 planeColor(0.5f, 0.5f, 0.5f);

struct Material
{
    glm::vec3 ambient=glm::vec3(1.0f, 0.5f, 0.31f);
    glm::vec3 diffuse=glm::vec3(1.0f, 0.5f, 0.31f);
    glm::vec3 specular=glm::vec3(0.5f, 0.5f, 0.5f);
    float shininess = 32.0f;
};

struct DirLight
{
	glm::vec3 direction = glm::vec3(2.0f, 3.0f, 2.0f);

    glm::vec3 ambient = glm::vec3(0.1f);
    glm::vec3 diffuse = glm::vec3(0.5f);
    glm::vec3 specular = glm::vec3(1.0f);
};

struct PointLight
{
    glm::vec3 position = glm::vec3(2.0f, 3.0f, 2.0f);

    glm::vec3 ambient = glm::vec3(0.1f);
    glm::vec3 diffuse = glm::vec3(0.5f);
    glm::vec3 specular = glm::vec3(1.0f);

    ///衰减系数
    float constant = 1.0;
    float linear = 0.09;
    float quadratic = 0.032;
};


struct SpotLight
{
    glm::vec3 position = glm::vec3(2.0f, 3.0f, 2.0f);
    glm::vec3 spotDir = normalize((glm::vec3(0.0) - position));
    float theta = cos(glm::radians(10.0f));

    glm::vec3 ambient = glm::vec3(0.1f);
    glm::vec3 diffuse = glm::vec3(0.5f);
    glm::vec3 specular = glm::vec3(1.0f);

    ///衰减系数
    float constant = 1.0;
    float linear = 0.09;
    float quadratic = 0.032;
};

//=(1.0f, 0.5f, 0.31f);
//glm::vec3 cubeColor(1.0f, 0.5f, 0.31f);
//glm::vec3 cubeColor(1.0f, 0.5f, 0.31f);

int main()
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    //创建一个面板
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "chapter02", nullptr, nullptr);
    if (window == nullptr)
    {
        std::cout << "Failed to create window" << std::endl;
        glfwTerminate();
        return -1;
    }
    //设置关联环境
    glfwMakeContextCurrent(window);

    //设置三个回调函数
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callBack);
    glfwSetScrollCallback(window, scroll_callBack);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    //聚焦光标
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    //stbi_set_flip_vertically_on_load(true);

    //开启深度测试,从而显示空间结构
    glEnable(GL_DEPTH_TEST);
    //glDepthFunc(GL_LESS);
    //glEnable(GL_STENCIL_TEST);
    //glStencilMask(0x00);

    //启用混合
    /*glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);*/

    //glEnable(GL_CULL_FACE);
    ////剔除面片，默认为背面
    //glCullFace(GL_FRONT);

    float vertices[] = {
        // positions          // normals           // texture coords
        -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f, 0.0f,
         0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f, 0.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f, 1.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f, 1.0f,
        -0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f, 1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f, 0.0f,

        -0.5f, -0.5f,  0.5f,  0.0f,  0.0f, 1.0f,   0.0f, 0.0f,
         0.5f, -0.5f,  0.5f,  0.0f,  0.0f, 1.0f,   1.0f, 0.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  0.0f, 1.0f,   1.0f, 1.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  0.0f, 1.0f,   1.0f, 1.0f,
        -0.5f,  0.5f,  0.5f,  0.0f,  0.0f, 1.0f,   0.0f, 1.0f,
        -0.5f, -0.5f,  0.5f,  0.0f,  0.0f, 1.0f,   0.0f, 0.0f,

        -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  1.0f, 0.0f,
        -0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  1.0f, 1.0f,
        -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  0.0f, 1.0f,
        -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  0.0f, 1.0f,
        -0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  0.0f, 0.0f,
        -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  1.0f, 0.0f,

         0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f,
         0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f,
         0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f, 1.0f,
         0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f, 1.0f,
         0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  0.0f, 0.0f,
         0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f,

        -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  0.0f, 1.0f,
         0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  1.0f, 1.0f,
         0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  1.0f, 0.0f,
         0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  1.0f, 0.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  0.0f, 0.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  0.0f, 1.0f,

        -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  0.0f, 1.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  1.0f, 1.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  1.0f, 0.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  1.0f, 0.0f,
        -0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  0.0f, 0.0f,
        -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  0.0f, 1.0f
    };


    float planeVertices[] = {
        // positions          // texture Coords (note we set these higher than 1 (together with GL_REPEAT as texture wrapping mode). this will cause the floor texture to repeat)
         5.0f, -0.5f,  5.0f,  2.0f, 0.0f,
        -5.0f, -0.5f,  5.0f,  0.0f, 0.0f,
        -5.0f, -0.5f, -5.0f,  0.0f, 2.0f,

         5.0f, -0.5f,  5.0f,  2.0f, 0.0f,
        -5.0f, -0.5f, -5.0f,  0.0f, 2.0f,
         5.0f, -0.5f, -5.0f,  2.0f, 2.0f
    };

    float transparentVertices[] = {
        // positions         // texture Coords (swapped y coordinates because texture is flipped upside down)
        0.0f,  0.5f,  0.0f,  0.0f,  0.0f,
        0.0f, -0.5f,  0.0f,  0.0f,  1.0f,
        1.0f, -0.5f,  0.0f,  1.0f,  1.0f,

        0.0f,  0.5f,  0.0f,  0.0f,  0.0f,
        1.0f, -0.5f,  0.0f,  1.0f,  1.0f,
        1.0f,  0.5f,  0.0f,  1.0f,  0.0f
    };

    float quadVertices[] = { // vertex attributes for a quad that fills the entire screen in Normalized Device Coordinates.
        // positions   // texCoords
        -1.0f,  1.0f,  0.0f, 1.0f,
        -1.0f, -1.0f,  0.0f, 0.0f,
         1.0f, -1.0f,  1.0f, 0.0f,

        -1.0f,  1.0f,  0.0f, 1.0f,
         1.0f, -1.0f,  1.0f, 0.0f,
         1.0f,  1.0f,  1.0f, 1.0f
    };

/*    
    MyShader cubeShader("./Resource/shader.vs", "./Resource/shader.fs");
    MyShader lightShader("./Resource/lightshader.vs", "./Resource/lightshader.fs");

    Material cubeMaterial;

    DirLight    dirLight;
    PointLight  pointLight;
    SpotLight   spotLight;
		
    


    glm::vec3 cubePositions[] = {
        glm::vec3(0.0f,  0.0f,  0.0f),
        glm::vec3(2.0f,  5.0f, -15.0f),
        glm::vec3(-1.5f, -2.2f, -2.5f),
        glm::vec3(-3.8f, -2.0f, -12.3f),
        glm::vec3(2.4f, -0.4f, -3.5f),
        glm::vec3(-1.7f,  3.0f, -7.5f),
        glm::vec3(1.3f, -2.0f, -2.5f),
        glm::vec3(1.5f,  2.0f, -2.5f),
        glm::vec3(1.5f,  0.2f, -1.5f),
        glm::vec3(-1.3f,  1.0f, -1.5f)
    };

    unsigned int cubeVAO, VBO;

    glGenVertexArrays(1, &cubeVAO);
    //绑定数据,生成缓冲区对象的名称,VBO指向该缓冲对象，函数会在buffers里返回n个缓冲对象的名称
    glGenBuffers(1, &VBO);

    //告诉VBO是个顶点缓冲类型，激活该类型
    //允许我们同一时间绑定多个不同类型的对象，不允许同时绑定相同类型的数据
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    //传入具体的数据，注意一定要先绑定好对象，这是过程式编程的理念
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);


    //绑定顶点缓冲对象
    glBindVertexArray(cubeVAO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);


    unsigned int lightVAO;

    glGenVertexArrays(1, &lightVAO);
    //绑定顶点缓冲对象
    glBindVertexArray(lightVAO);

    //绑定数据
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);


    unsigned int diffuseMap;
    unsigned int specularMap;
    unsigned int emissionMap;

    MyShader::loadTexture("./Resource/container2.jpg", diffuseMap);

    MyShader::loadTexture("./Resource/container2_specular.jpg",specularMap);


    MyShader::loadTexture("./Resource/matrix.jpg", emissionMap);


    cubeShader.use();
    cubeShader.setInt("material.diffuse", 0);
    cubeShader.setInt("material.specular", 1);
    cubeShader.setInt("material.emission", 2);


   

    auto path = "./Resource/matrix.jpg";
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_FlipUVs);

*/
	MyShader shader("./Shader/model_loading.vs", "./Shader/model_loading.fs");
	camera.reset(new Camera(window, SCR_WIDTH, SCR_HEIGHT));

    Model modelLoader("./Resource/nanosuit/nanosuit.obj");
    MyShader lightShader("./Shader/lightshader.vs", "./Shader/lightshader.fs");

    MyShader plantSingleColor("./Shader/shaderSingleColor.vs", "./Shader/shaderSingleColor.fs");

    MyShader grassShader("./Shader/grassBlend.vs", "./Shader/grassBlend.fs");

    MyShader frameShader("./Shader/frameBuffer.vs", "./Shader/frameBuffer.fs");
     
    unsigned int lightVAO;
    unsigned int VBO;

    glGenVertexArrays(1, &lightVAO);
    //绑定顶点缓冲对象
    glBindVertexArray(lightVAO);

    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    //传入具体的数据，注意一定要先绑定好对象，这是过程式编程的理念
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    //绑定数据
    //glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // cube VAO
    unsigned int cubeVAO, cubeVBO;
    glGenVertexArrays(1, &cubeVAO);
    glGenBuffers(1, &cubeVBO);
    glBindVertexArray(cubeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), &vertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glBindVertexArray(0);

    // plane VAO
    unsigned int planeVAO, planeVBO;
    glGenVertexArrays(1, &planeVAO);
    glGenBuffers(1, &planeVBO);
    glBindVertexArray(planeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(planeVertices), &planeVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glBindVertexArray(0);


    // transparent VAO
    unsigned int transparentVAO, transparentVBO;
    glGenVertexArrays(1, &transparentVAO);
    glGenBuffers(1, &transparentVBO);
    glBindVertexArray(transparentVAO);
    glBindBuffer(GL_ARRAY_BUFFER, transparentVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(transparentVertices), transparentVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glBindVertexArray(0);

    //quadVAO
    unsigned int quadVAO, quadVBO;
    glGenVertexArrays(1, &quadVAO);
    glGenBuffers(1, &quadVBO);
    glBindVertexArray(quadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glBindVertexArray(0);

    unsigned int transparentTexture;
    MyShader::loadTexture("./Resource/transWindow.png", transparentTexture);

    unsigned int boxTexture;
    MyShader::loadTexture("./Resource/container.jpg", boxTexture);

    std::vector<glm::vec3> vegetation
    {
        glm::vec3(-1.5f, 0.0f, -0.48f),
        glm::vec3(1.5f, 0.0f, 0.51f),
        glm::vec3(0.0f, 0.0f, 0.7f),
        glm::vec3(-0.3f, 0.0f, -2.3f),
        glm::vec3(0.5f, 0.0f, -0.6f)
    };  
    const auto& cameraPos = camera->getCameraPos();
    
    //帧缓冲
    if (false)
    {
        unsigned int fbo;
        glGenFramebuffers(1, &fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        //创建纹理
        unsigned int texture;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 800, 600, 0, GL_RGB, GL_UNSIGNED_INT, nullptr);

        glTextureParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTextureParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        //当纹理大小与渲染窗口大小不一致时，需要调用glViewPort
        //附加到帧
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);

        //也可以将深度缓冲和模板缓冲附加为一个单独的纹理
        {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH24_STENCIL8, 800, 600, 0, GL_DEPTH_STENCIL, GL_UNSIGNED_INT_24_8, nullptr);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_TEXTURE_2D, texture, 0);
        }

        if (glCheckFramebufferStatus(fbo) == GL_FRAMEBUFFER_COMPLETE)
        {

        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glDeleteFramebuffers(1, &fbo);
        //创建渲染缓冲对象
        {
            unsigned int rbo;
            glGenRenderbuffers(1, &rbo);
            glBindRenderbuffer(GL_RENDERBUFFER, rbo);

            glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, 800, 600);
            glFramebufferRenderbuffer(GL_RENDERBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);
        }
    }

    unsigned int framebuffer;
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    // 生成纹理
    unsigned int texColorBuffer;
    glGenTextures(1, &texColorBuffer);
    glBindTexture(GL_TEXTURE_2D, texColorBuffer);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, SCR_WIDTH, SCR_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    //glBindTexture(GL_TEXTURE_2D, 0);

    // 将它附加到当前绑定的帧缓冲对象
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texColorBuffer, 0);


    unsigned int rbo;
    glGenRenderbuffers(1, &rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, SCR_WIDTH, SCR_HEIGHT);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)[[_LIKELY]]
        std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    float windowScale = 1.0;
    while (!glfwWindowShouldClose(window))
    {
        processInput(window);
        float timeValue = glfwGetTime();
        camera->setMoveSpeed(timeValue);
        glm::mat4 view, projection;
        view = camera->getViewMat();
        projection = camera->getProjectionMat();


        
        glViewport((int)SCR_WIDTH * (1.-windowScale)/2.0, (int)SCR_WIDTH * (1. - windowScale) / 2.0, 
            (int)SCR_WIDTH * windowScale, (int)SCR_HEIGHT * windowScale);
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // 我们现在不使用模板缓冲
        glEnable(GL_DEPTH_TEST);
       


        shader.use();
        shader.setMat4("projection", projection);
        shader.setMat4("view", view);

        //使用线条模式绘制
        //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, glm::vec3(0.0f, 0.0f, 0.0f)); // translate it down so it's at the center of the scene
        model = glm::scale(model, glm::vec3(1.0f, 1.0f, 1.0f));	// it's a bit too big for our scene, so scale it down
        shader.setMat4("model", model);
        //modelLoader.Draw(shader);
        //填充模式绘制
        //glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);


        lightShader.use();
        glm::mat4 lightModel;
        lightModel = glm::mat4(1.0f);
        lightModel = glm::translate(lightModel, glm::vec3(1.0f));
        lightModel = glm::scale(lightModel, glm::vec3(0.2f));
        lightModel = glm::rotate(lightModel, timeValue, glm::vec3(0.0f, 1.0f, 0.0f));
 
        lightShader.setVec3("lightColor", lightColor);
        lightShader.setMat4("model", lightModel);
        lightShader.setMat4("view", view);
        lightShader.setMat4("projection", projection);
        
        glBindVertexArray(lightVAO);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        ///放置一个箱子,但沿用grassShader
        grassShader.use();
        glBindVertexArray(cubeVAO);
        glBindTexture(GL_TEXTURE_2D, boxTexture);
        glm::mat4 boxModel;
        boxModel = glm::mat4(1.0f);
        boxModel = glm::translate(boxModel, glm::vec3(0.0f));
        boxModel = glm::scale(boxModel, glm::vec3(0.2f));
        boxModel = glm::rotate(boxModel, timeValue, glm::vec3(0.0f, 1.0f, 0.0f));

        grassShader.setMat4("model", boxModel);
        grassShader.setMat4("view", view);
        grassShader.setMat4("projection", projection);

        //glBindVertexArray(lightVAO);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        std::sort(vegetation.begin(), vegetation.end(), [cameraPos](const glm::vec3& l, const glm::vec3& r)
            {
                return glm::length(l.z - cameraPos.z) > glm::length(r.z - cameraPos.z);
            });
        grassShader.use();
        //启用混合
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glBindVertexArray(transparentVAO);
        glBindTexture(GL_TEXTURE_2D, transparentTexture);
        grassShader.setMat4("view", view);
        grassShader.setMat4("projection", projection);
        for (unsigned int i = 0; i < vegetation.size(); i++)
        {
            model = glm::translate(model, vegetation[i]);
            grassShader.setMat4("model", model);
            
            glDrawArrays(GL_TRIANGLES, 0, 6);
        }
        glDisable(GL_BLEND);



        // 第二处理阶段
        //glViewport(0, 0, SCR_WIDTH, SCR_HEIGHT);
        glBindFramebuffer(GL_FRAMEBUFFER, 0); // 返回默认
        glEnable(GL_DEPTH_TEST);
        glClearColor(0.5f, 0.5f, 0.5f, 0.5f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        ///放置一个箱子,但沿用frameShader
        frameShader.use();
        glBindVertexArray(quadVAO);
        glBindTexture(GL_TEXTURE_2D, texColorBuffer);

        glm::mat4 frameModel;
        frameModel = glm::mat4(1.0f);
        frameShader.setMat4("view", view);
        frameShader.setMat4("projection", projection);
        frameShader.setMat4("model", frameModel);
        

        glDrawArrays(GL_TRIANGLES, 0, 36);


        ///放置一个箱子,但沿用grassShader
        grassShader.use();
        glBindVertexArray(cubeVAO);
        glBindTexture(GL_TEXTURE_2D, boxTexture);
        //glm::mat4 boxModel;
        boxModel = glm::mat4(1.0f);
        boxModel = glm::translate(boxModel, glm::vec3(1.0f));
        boxModel = glm::scale(boxModel, glm::vec3(0.5f));
        boxModel = glm::rotate(boxModel, timeValue, glm::vec3(0.0f, 0.0f, 1.0f));

        grassShader.setMat4("model", boxModel);
        grassShader.setMat4("view", view);
        grassShader.setMat4("projection", projection);

        //glBindVertexArray(lightVAO);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        //glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);


        //glEnable(GL_STENCIL_TEST);
        //glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
        //glStencilFunc(GL_ALWAYS, 1, 0xFF); // 所有的片段都应该更新模板缓冲
        //glStencilMask(0xFF); // 启用模板缓冲写入

        //glm::mat4 planeModel;
        //lightShader.use();
        //planeModel = glm::mat4(1.0f);
        //lightShader.setVec3("lightColor", planeColor);
        //lightShader.setMat4("model", planeModel);
        //lightShader.setMat4("view", view);
        //lightShader.setMat4("projection", projection);

        //glBindVertexArray(planeVAO);
        //glDrawArrays(GL_TRIANGLES, 0, 6);

        //glStencilFunc(GL_NOTEQUAL, 1, 0xFF);
        //glStencilMask(0x00); // 禁止模板缓冲的写入
        ////glDisable(GL_DEPTH_TEST);
        //plantSingleColor.use();

        ////DrawTwoScaledUpContainers();
        //glm::mat4 planeModelUp;
        //planeModelUp = glm::mat4(1.0f);
        //planeModelUp = glm::scale(planeModelUp, glm::vec3(1.01f));
        //plantSingleColor.setVec3("lightColor", planeColor);
        //plantSingleColor.setMat4("model", planeModelUp);
        //plantSingleColor.setMat4("view", view);
        //plantSingleColor.setMat4("projection", projection);
        //glBindVertexArray(planeVAO);
        //glDrawArrays(GL_TRIANGLES, 0, 6);

        //glStencilMask(0xFF);
        //glEnable(GL_DEPTH_TEST);
        //glDisable(GL_STENCIL_TEST);
        //glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    camera->processInput();
}

void mouse_callBack(GLFWwindow* window, double xPos, double yPos)
{
    camera->mouse_callBack(xPos, yPos);
}

void scroll_callBack(GLFWwindow* windos, double xOffset, double yOffset)
{
    camera->scroll_callBack(xOffset, yOffset);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}