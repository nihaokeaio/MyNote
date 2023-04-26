```c++
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <iostream>

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow* window);

// settings
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

int main()
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }


    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    //创建三角形三维端点，由于平面为二维，故第三个参数暂时无效
    float vertices[] = {
        // positions         // colors
         0.5f, -0.5f, 0.0f,  1.0f, 0.0f, 0.0f,   // bottom right
        -0.5f, -0.5f, 0.0f,  0.0f, 1.0f, 0.0f,   // bottom left
         0.0f,  0.5f, 0.0f,  0.0f, 0.0f, 1.0f    // top 
    };
    unsigned int VAO;
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);

    //创建端点缓冲对象
    unsigned int VBO;
    glGenBuffers(1, &VBO);
    //OPGL有多种不同类型的缓冲器，这里端点我们使用GL_ARRAY_BUFFER
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    
    //使用glBufferData函数将端点数据放入缓冲区，最后一个参数有三种类型
    //GL_STREAM_DRAW 只设置一次（不频繁变动）并且极少被GPU使用
    //GL_STATIC_DRAW 设置一次，经常被使用
    //GL_DYNAMIC_DRAW 频繁变动，经常变动（加载至内存中，降低延迟）
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    //将点着色器语言GLSL （OpenGL Shading Language）以c字符串形式存储。
    //此处将vertexColor设置为输出，作为下一个着色器的输入
    const char* vertexShaderSource = "#version 330 core\n"
        "layout (location = 0) in vec3 aPos;\n"
        "layout (location = 1) in vec3 aColor;\n"
        "out vec3 ourColor;\n"
        "void main()\n"
        "{\n"
        "   gl_Position = vec4(aPos, 1.0);\n"
        "   ourColor=aColor;\n"
        "}\0";
    //通过ID号创建着色器对象，这里使用点着色器GL_VERTEX_SHADER
    unsigned int vertexShader;
    vertexShader = glCreateShader(GL_VERTEX_SHADER);

    //将着色器源代码附加到着色器对象
    glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
    //编译GLSL语言
    glCompileShader(vertexShader);

    //使用如下方式可以检验编译是否成功
    /*
    int  success;
    char infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    
    if(!success)
    {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
    }
    //如果编译有错误则会将错误信息保存在日志中
    */
    //此处vertexColor作为前一个着色器的输入

    //同样，我们可以考虑使用uniform类型（全局变量，可以实现CPU和GPU的通信）
    const char* fragmentShaderSource = "#version 330 core\n"
        "out vec4 FragColor;\n"
        "in vec3 ourColor;\n"
        "uniform vec4 myColor;\n"
        "void main()\n"
        "{\n"
        "FragColor=vec4(ourColor,1.0f);\n"
        "}\0";

    unsigned int fragmentShader;
    fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
    glCompileShader(fragmentShader);

    //创建着色器程序，开始链接着色器对象
    unsigned int shaderProgram;
    shaderProgram = glCreateProgram();

    //链接，前者链接的输出是后者连接的输入
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    /*
    * //使用同样的方法可以验证链接是否正确
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if(!success) 
    {
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        ...
    }
    */
    

    

    //现在该开始告诉OpenGL应该怎么解释我们的数据了
    /*
    0:与前文location=0相对应
    3:端点的属性，这是一个三维的端点
    GL_FLOAT:端点为float类型
    GL_FALSE:是否归一化，这里我们不考虑将float数据归一化为int类型
    3 * sizeof(float)：端点间距，三个float大小
    (void*)0：位置数据在缓冲区中开始的偏移量
    */
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    //启用端点属性
    glEnableVertexAttribArray(0);

    //配置颜色属性
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3*sizeof(float)));
    //启用端点属性
    glEnableVertexAttribArray(1);

    



    // render loop
    // -----------
    while (!glfwWindowShouldClose(window))
    {
        // input
        // -----
        processInput(window);

        // render
        // ------
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        //获得系统时间作为变量,这三条语句可以在非循环语句外事先完成
        float timeValue = glfwGetTime();
        float greenValue = (sin(timeValue) / 2.0) + 0.5f;
        //首先需要得到uniform变量，使用glGetUniformLocation函数。
        int vertexColorLocation = glGetUniformLocation(shaderProgram, "myColor");


        //激活（active）着色器程序
        glUseProgram(shaderProgram);
        //设置uniform变量值
        glUniform4f(vertexColorLocation, 0.0f, greenValue, 0.0f, 1.0f);

        glBindVertexArray(VAO);
        //0：顶点起始索引，3：绘制三个顶点（正好有三个）
        glDrawArrays(GL_TRIANGLES, 0, 3);


        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    //别忘记删除着色器对象，现在已经用不到他们了
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    glfwTerminate();
    return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}
```
*****

## 该解释的反正都已经在代码里注释好了。