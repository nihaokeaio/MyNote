#include <glad/glad.h>
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


void getRandModel(float time, std::vector<glm::mat4>& modelMatrices)
{
    srand(glfwGetTime()); // 初始化随机种子    
    float radius = 50.0;
    float offset = 2.5f;
    int n = modelMatrices.size();
    for (unsigned int i = 0; i < n; i++)
    {
        glm::mat4 model = glm::mat4(1.0f);
        // 1. 位移：分布在半径为 'radius' 的圆形上，偏移的范围是 [-offset, offset]
        float angle = (float)i / (float)n * 360.0f;
        float displacement = (rand() % (int)(2 * offset * 100)) / 100.0f - offset;
        float x = sin(angle) * radius + displacement;
        displacement = (rand() % (int)(2 * offset * 100)) / 100.0f - offset;
        float y = displacement * 0.4f; // 让行星带的高度比x和z的宽度要小
        displacement = (rand() % (int)(2 * offset * 100)) / 100.0f - offset;
        float z = cos(angle) * radius + displacement;
        model = glm::translate(model, glm::vec3(x, y, z));

        // 2. 缩放：在 0.05 和 0.25f 之间缩放
        float scale = (rand() % 20) / 100.0f + 0.05;
        model = glm::scale(model, glm::vec3(scale));

        // 3. 旋转：绕着一个（半）随机选择的旋转轴向量进行随机的旋转
        float rotAngle = (rand() % 360);
        model = glm::rotate(model, rotAngle, glm::vec3(0.4f, 0.6f, 0.8f));

        // 4. 添加到矩阵的数组中
        modelMatrices[i] = model;
    }
}
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

    float skyboxVertices[] = {
        // positions          
        -1.0f,  1.0f, -1.0f,
        -1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,
         1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f,

        -1.0f, -1.0f,  1.0f,
        -1.0f, -1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f,  1.0f,
        -1.0f, -1.0f,  1.0f,

         1.0f, -1.0f, -1.0f,
         1.0f, -1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f,  1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,

        -1.0f, -1.0f,  1.0f,
        -1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f, -1.0f,  1.0f,
        -1.0f, -1.0f,  1.0f,

        -1.0f,  1.0f, -1.0f,
         1.0f,  1.0f, -1.0f,
         1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,
        -1.0f,  1.0f,  1.0f,
        -1.0f,  1.0f, -1.0f,

        -1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f,
         1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f,
         1.0f, -1.0f,  1.0f
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


    //float points[] = {
   float points[] = {
    -0.5f,  0.5f, 1.0f, 0.0f, 0.0f, // 左上
     0.5f,  0.5f, 0.0f, 1.0f, 0.0f, // 右上
     0.5f, -0.5f, 0.0f, 0.0f, 1.0f, // 右下
    -0.5f, -0.5f, 1.0f, 1.0f, 0.0f  // 左下
    };

   float quadIntsVertices[] = {
       // 位置          // 颜色
       -0.05f,  0.05f,  1.0f, 0.0f, 0.0f,
        0.05f, -0.05f,  0.0f, 1.0f, 0.0f,
       -0.05f, -0.05f,  0.0f, 0.0f, 1.0f,

       -0.05f,  0.05f,  1.0f, 0.0f, 0.0f,
        0.05f, -0.05f,  0.0f, 1.0f, 0.0f,
        0.05f,  0.05f,  0.0f, 1.0f, 1.0f
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
	camera.reset(new Camera(window, SCR_WIDTH, SCR_HEIGHT));


	MyShader shader("./Shader/model_loading.vs", "./Shader/model_loading.fs");
    shader.attachShader("./Shader/model_loading.gs", GL_GEOMETRY_SHADER);

    Model modelLoader("./Resource/nanosuit/nanosuit.obj");

    Model plantLoader("./Resource/planet/planet.obj");
    Model rockLoader("./Resource/rock/rock.obj");

    MyShader lightShader("./Shader/lightshader.vs", "./Shader/lightshader.fs");

    MyShader plantSingleColor("./Shader/shaderSingleColor.vs", "./Shader/shaderSingleColor.fs");

    MyShader grassShader("./Shader/grassBlend.vs", "./Shader/grassBlend.fs");

    MyShader frameShader("./Shader/frameBuffer.vs", "./Shader/frameBuffer.fs");

    MyShader skyBoxShader("./Shader/skybox.vs", "./Shader/skybox.fs");

    MyShader reflectBoxShader("./Shader/reflectBox.vs", "./Shader/reflectBox.fs");

    MyShader pointsShader("./Shader/pointsShader.vs", "./Shader/pointsShader.fs");
    pointsShader.attachShader("./Shader/pointsShader.gs", GL_GEOMETRY_SHADER);

    MyShader normalShader("./Shader/normalShader.vs", "./Shader/normalShader.fs");
    normalShader.attachShader("./Shader/normalShader.gs", GL_GEOMETRY_SHADER);

    MyShader qurdIntsShader("./Shader/qurdInts.vs", "./Shader/qurdInts.fs");

    MyShader qurdRockIntsShader("./Shader/qurdPlantInts.vs", "./Shader/qurdPlantInts.fs");

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
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), nullptr, GL_STATIC_DRAW);
    
    ///使用glBufferSubData进行部分填充
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
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


    //skyBoxVAO
    unsigned int skyBoxVAO, skyBoxVBO;
    glGenVertexArrays(1, &skyBoxVAO);
    glGenBuffers(1, &skyBoxVBO);
    glBindVertexArray(skyBoxVAO);
    glBindBuffer(GL_ARRAY_BUFFER, skyBoxVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glBindVertexArray(0);

    // points VAO
    unsigned int pointVAO, pointVBO;
    glGenVertexArrays(1, &pointVAO);
    glGenBuffers(1, &pointVBO);
    glBindVertexArray(pointVAO);
    glBindBuffer(GL_ARRAY_BUFFER, pointVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(points), &points, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(2 * sizeof(float)));
    glBindVertexArray(0);

    // points VAO
    unsigned int qurdIntsVAO, qurdIntsVBO;
    glGenVertexArrays(1, &qurdIntsVAO);
    glGenBuffers(1, &qurdIntsVBO);
    glBindVertexArray(qurdIntsVAO);
    glBindBuffer(GL_ARRAY_BUFFER, qurdIntsVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadIntsVertices), &quadIntsVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(2 * sizeof(float)));
    glBindVertexArray(0);

    unsigned int transparentTexture;
    MyShader::loadTexture("./Resource/transWindow.png", transparentTexture);

    unsigned int boxTexture;
    MyShader::loadTexture("./Resource/container.jpg", boxTexture);
    
    std::vector<std::string>cubeFileName =
    {
        "./Resource/skybox/right.jpg",
        "./Resource/skybox/left.jpg",
        "./Resource/skybox/top.jpg",
        "./Resource/skybox/bottom.jpg",
        "./Resource/skybox/front.jpg",
        "./Resource/skybox/back.jpg",
    };

    unsigned int skyBoxTexture;
    MyShader::loadCubeTexture(cubeFileName, skyBoxTexture);


    std::vector<glm::vec3> vegetation
    {
        glm::vec3(-1.5f, 0.0f, -0.48f),
        glm::vec3(1.5f, 0.0f, 0.51f),
        glm::vec3(0.0f, 0.0f, 0.7f),
        glm::vec3(-0.3f, 0.0f, -2.3f),
        glm::vec3(0.5f, 0.0f, -0.6f)
    };  

    glm::vec2 translations[100];
    int index = 0;
    float offset = 0.1f;
    for (int y = -10; y < 10; y += 2)
    {
        for (int x = -10; x < 10; x += 2)
        {
            glm::vec2 translation;
            translation.x = (float)x / 10.0f + offset;
            translation.y = (float)y / 10.0f + offset;
            translations[index++] = translation;
        }
    }

    unsigned int instanceVBO;
    glBindVertexArray(qurdIntsVAO);
    glGenBuffers(1, &instanceVBO);
    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec2) * 100, &translations[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)(0));
    //glBindBuffer(GL_ARRAY_BUFFER, 0);
    glVertexAttribDivisor(2, 1);
    glBindVertexArray(0);


    
    std::vector<glm::mat4> modelMatrices(20000, glm::mat4());
    getRandModel(glfwGetTime(), modelMatrices);
    rockLoader.attachAttribPointer(modelMatrices);
   

    unsigned int instancePlantVBO;
    glBindVertexArray(qurdIntsVAO);
    glGenBuffers(1, &instancePlantVBO);
    glBindBuffer(GL_ARRAY_BUFFER, instancePlantVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec2) * 100, &translations[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)(0));
    //glBindBuffer(GL_ARRAY_BUFFER, 0);
    glVertexAttribDivisor(2, 1);
    glBindVertexArray(0);

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
        modelLoader.Draw(shader);
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
        reflectBoxShader.use();
        glBindVertexArray(cubeVAO);
        glBindTexture(GL_TEXTURE_2D, boxTexture);
        //glBindTexture(GL_TEXTURE_CUBE_MAP, skyBoxTexture);
        //glm::mat4 boxModel;

        boxModel = glm::mat4(1.0f);
        boxModel = glm::translate(boxModel, glm::vec3(1.0f));
        boxModel = glm::scale(boxModel, glm::vec3(0.5f));
        //boxModel = glm::rotate(boxModel, timeValue, glm::vec3(0.0f, 0.0f, 1.0f));

        reflectBoxShader.setMat4("model", boxModel);
        reflectBoxShader.setMat4("view", view);
        reflectBoxShader.setMat4("projection", projection);
        reflectBoxShader.setVec3("cameraPos", camera->getCameraPos());
        glDrawArrays(GL_TRIANGLES, 0, 36);

        //glBindVertexArray(lightVAO);
        normalShader.use();
        glBindVertexArray(cubeVAO);
        normalShader.setMat4("model", boxModel);
        normalShader.setMat4("view", view);
        normalShader.setMat4("projection", projection);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        //glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        reflectBoxShader.use();
        reflectBoxShader.setMat4("projection", projection);
        reflectBoxShader.setMat4("view", view);

        model = glm::mat4(1.0f);
        model = glm::translate(model, glm::vec3(0.0f, 0.0f, 0.0f)); // translate it down so it's at the center of the scene
        model = glm::scale(model, glm::vec3(1.0f, 1.0f, 1.0f));	// it's a bit too big for our scene, so scale it down
        reflectBoxShader.setMat4("model", model);
        //modelLoader.Draw(reflectBoxShader);
        normalShader.use();
        normalShader.setMat4("model", model);
        //modelLoader.Draw(normalShader);


        pointsShader.use();
        glBindVertexArray(pointVAO);
        //glDrawArrays(GL_POINTS, 0, 4);


        shader.use();
        //使用线条模式绘制
        model = glm::mat4(1.0f);
        model = glm::translate(model, glm::vec3(2.0f, 0.0f, 0.0f)); // translate it down so it's at the center of the scene
        model = glm::scale(model, glm::vec3(0.3f, 0.3f, 0.3f));	// it's a bit too big for our scene, so scale it down
        shader.setMat4("model", model);
        shader.setMat4("projection", projection);
        shader.setMat4("view", view);
        shader.setFloat("time", timeValue);
        modelLoader.Draw(shader);

        
        qurdIntsShader.use();
        //for (unsigned int i = 0; i < 100; i++)
        //{
        //    //DoSomePreparations(); // 绑定VAO，绑定纹理，设置uniform等

        //     glBindVertexArray(qurdIntsVAO);
        //    //qurdIntsShader.setVec2(("offsets[" + std::to_string(i) + "]").c_str(), translations[i]);
        //}   
        glBindVertexArray(qurdIntsVAO);
        glDrawArraysInstanced(GL_TRIANGLES, 0, 6, 100);
        

        shader.use();
        model = glm::translate(model, glm::vec3(0.0f, -3.0f, 0.0f));
        model = glm::scale(model, glm::vec3(10.0f, 10.0f, 10.0f));
        shader.setMat4("model", model);
        plantLoader.Draw(shader);

        // 绘制小行星
        /*shader.use();
        for (unsigned int i = 0; i < modelMatrices.size(); i++)
        {
            shader.setMat4("model", modelMatrices[i]);
            rockLoader.Draw(shader);
        }*/
        qurdRockIntsShader.use();
        model = glm::mat4(1.0);
        qurdRockIntsShader.setMat4("projection", projection);
        qurdRockIntsShader.setMat4("view", view);
        rockLoader.DrawInts(qurdRockIntsShader, modelMatrices.size());

        ///放置一个skyBox,使用skyBoxShader
        //glDepthFunc(GL_LEQUAL);
        //skyBoxShader.use();
        //glBindVertexArray(skyBoxVAO);
        //glBindTexture(GL_TEXTURE_CUBE_MAP, skyBoxTexture);
        ////glm::mat4 boxModel;
        //boxModel = glm::mat4(1.0f);
        ////移除摄像机的位移信息
        //view = glm::mat4(glm::mat3(camera->getViewMat()));

        //skyBoxShader.setMat4("model", boxModel);
        //skyBoxShader.setMat4("view", view);
        //skyBoxShader.setMat4("projection", projection);

        ////glBindVertexArray(lightVAO);
        //glDrawArrays(GL_TRIANGLES, 0, 36);
        //glDepthFunc(GL_LESS);
        

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