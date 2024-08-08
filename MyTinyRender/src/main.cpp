#include <iostream>
#include "Vector.hpp"
#include "Scene.h"
#include "global.hpp"
#include "Geometry.h"
#include "LoaderMesh.h"
#include "Texture.h"
#include  "Camera.h"
#include  <Timer.h>
#include  <ctime>

struct light
{
    Vec3f position;
    Vec3f intensity;
};


//设置渲染方式
Vec3f rstShaderFunUseTexture(const fragment_shader_payload& fsp)
{
    Vec3f textureColor;
    if (fsp.mesh->textures_.empty())
    {
        textureColor = fsp.color;
        return textureColor;
    }

    for (const auto& item : fsp.mesh->textures_)
    {
        textureColor += item->getColorBilinear(fsp.tex_coords);
    }   
    return textureColor / 255.f;
}

//设置渲染方式
Vec3f rstShaderFunUseColor(const fragment_shader_payload& fsp)
{
    Vec3f textureColor = fsp.color;
    
    return textureColor;
}

Vec3f rstShaderFunUseBlinnPhong(const fragment_shader_payload& fsp)
{
    Vec3f color;
    auto texture_color = rstShaderFunUseTexture(fsp);
    
    float p = 150;

    Vec3f ka = Vec3f(0.005, 0.005, 0.005);
    Vec3f kd = texture_color;
    Vec3f ks = Vec3f(0.7937, 0.7937, 0.7937);


    auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
    auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

    std::vector<light> lights = { l1, l2 };
    Vec3f amb_light_intensity{ 10, 10, 10 };

    for (auto& light : lights)
    {
        Vec3f viewDir = -fsp.view_dir;
        Vec3f lightDir = light.position - fsp.point;
        Vec3f normal = fsp.normal;

        float r2 = lightDir.norm2();
        float value = normal.dot(lightDir.normalize());
        Vec3f Ld = Vec3f::vecMultiplication(kd, light.intensity) / r2 * MAX(value, 0);
        Vec3f h = ((lightDir + viewDir) / 2).normalize();
        Vec3f Ls = Vec3f::vecMultiplication(ks, light.intensity) / r2 * pow(MAX(normal.dot(h), 0), p);

        Vec3f La = Vec3f::vecMultiplication(ka, amb_light_intensity);
        color += (Ld + Ls + La);
    }


    return color;
}


inline void mouseCallBack(int event, int x, int y, int flags, void* param)
{
    auto scene = static_cast<Scene*>(param);
    if (scene)
    {
        scene->getCamera()->mouseCallBack(event, x, y, flags, param);
    }
}

void drawMesh(Scene* scene)
{
    

    const std::string& filePath = "./Resource/models/cube/cube.obj";
    const std::string& textureFilePath = "./Resource/models/cube/wall1.tif";
    const std::string& filePathSpot = "./Resource/models/spot/spot_triangulated_good.obj";
    const std::string& textureFilePathSpot = "./Resource/models/spot/spot_texture.png";


    std::shared_ptr<Texture> texture(new Texture(textureFilePath));
    std::shared_ptr<Texture> textureSpot(new Texture(textureFilePathSpot));
    LoaderMesh loader;
    
    /*loader.loadObjFile(filePath);

    for (auto mesh : loader.LoadedMeshes_)
    {
        mesh->loadTexture(texture);
        mesh->setRenderWay(Mesh::USE_TEXTURE);
		scene->addMesh(mesh);
    }*/
    loader.loadObjFile(filePathSpot);
    for (auto mesh : loader.LoadedMeshes_)
    {
        mesh->loadTexture(textureSpot);
        mesh->setRenderWay(Mesh::USE_TEXTURE);
        mesh->setModelMat(140, 2.5);
        scene->addMesh(mesh);
    }
}

void drawTriangle(Scene* scene)
{
   /* Triangle t0 = { Vec3f(-0.5, 0.5,0),   Vec3f(0.5, 0.5,0),  Vec3f(0, 0,-0) };
    Triangle t1 = { Vec3f(-0.2, 0.7,0),   Vec3f(0.6, 0.5,0),  Vec3f(-0.2, 0,-0) };
    Triangle t2 = { Vec3f(-0.1, 0.5,0),   Vec3f(0.2, 0.5,0),  Vec3f(0, 0,-0) };*/

   /* Triangle t0 = { {2, 0, -2},    {0, 2, -2},  {-2, 0, -2} };
    Triangle t1 = { {3.5, -1, -5},   {2.5, 1.5, -5 }, {-1, 0.5, -0} };*/
    float x = 5;
    float y = -5;
    float z = 5;
    Triangle t0 = { {-x, y, -z},   {-x, y, z},  {x, y, z} };
    Triangle t1 = { {x, y, -z},   {-x, y, -z }, {x, y, z} };

    Triangle t2 = { {1, 1, 1},   {-1, 1, 1},  {1, -1, 1} };
    Triangle t3 = { {-1, 1, 1},   {-1, -1, 1 }, {1, -1, 1} };

    Triangle t4 = { {1, 1, -1},   {-1, 1, -1},  {1, -1, -1} };
    Triangle t5 = { {-1, 1, -1},   {-1, -1, -1 }, {1, -1, -1} };

    Triangle t6 = { {-1, 1, 1},   {-1, 1, -1},  {-1, -1, 1} };
    Triangle t7 = { {-1, 1, -1},   {-1, -1, -1 }, {-1, -1, 1} };

    t0.setColor(0, Color::Red);
    t0.setColor(1, Color::Green);
    t0.setColor(2, Color::Blue);

    t1.setColor(0, Color::Green);
    t1.setColor(1, Color::Green);
    t1.setColor(2, Color::Blue);
    

    std::vector<Triangle*>geometries;
    geometries.push_back(&t0);
    geometries.push_back(&t1);
   /* geometries.push_back(&t2);
    geometries.push_back(&t3);
    geometries.push_back(&t4);
    geometries.push_back(&t5);
    geometries.push_back(&t6);
    geometries.push_back(&t7);*/

    std::shared_ptr<Mesh> mesh(new Mesh(geometries));
    scene->addMesh(mesh);
}

void drawLine(Scene* scene)
{
    Line l0 = { 13, 20, 120, 40 };
    Line l1 = { 20, 13, 40, 80 };
    Line l2 = { 80, 40, 13, 20 };
    std::vector<Geometry*>geometries;

    geometries.push_back(&l0);
    geometries.push_back(&l1);
    geometries.push_back(&l2);

    //scene->addModel(geometries);
}

void run(Scene* scene)
{
	drawMesh(scene);

    drawTriangle(scene);
    int key = 0;

    //cv::Mat image(scene->width_, scene->height_, CV_32FC3);
    //cv::imshow("image", image);
    //cv::setMouseCallback("image", &mouseCallBack, scene);
    bool isFirst = true;
    int frame_count = 0;
    scene->getCamera()->setMoveSpeed(-1);
    scene->setFragmentShaderFunction(&rstShaderFunUseBlinnPhong);
    while (key != 27) {
        Timer timer;
        scene->clear();
        scene->doUpDate();

        auto s = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()
        );

        scene->getCamera()->setMoveSpeed(static_cast<double>(s.count()));
        //scene->write();
        const auto pData = scene->getDataBuffer().data();
        cv::Mat image= cv::Mat(scene->height_, scene->width_, CV_32FC3, pData);

		cv::imshow("image", image);
		cv::setMouseCallback("image", &mouseCallBack, scene);
        //std::cout << scene->getCamera()->getCameraDir() << std::endl;;
        /*for(int i=0;i<scene->width_;++i)
        {
            for (int j = 0; j < scene->height_; ++j)
            {
                image.at<cv::Vec3f>(i, j)[0] = scene->getDataBuffer(i, j).z;
                image.at<cv::Vec3f>(i, j)[1] = scene->getDataBuffer(i, j).y;
                image.at<cv::Vec3f>(i, j)[2] = scene->getDataBuffer(i, j).x;
            }
        }*/
        scene->getCamera()->processInput();

        float ms = timer.stop();
        std::cout << "FPS = " << static_cast<float>(1000.0f / ms) << " \r";
        std::cout.flush();
    }
}

int main()
{
    Scene scene;
    scene.useMsaa(2, 2);

    run(&scene);
	
    scene.write();
	return 0;
};