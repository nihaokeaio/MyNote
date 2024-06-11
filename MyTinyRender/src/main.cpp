#include "Vector.hpp"
#include "Scene.h"
#include "global.hpp"
#include "Geometry.h"
#include "LoaderMesh.h"
#include "Texture.h"
#include  "Camera.h"
#include  <ctime>

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

    std::shared_ptr<Texture> texture(new Texture(textureFilePath));
    LoaderMesh loader(filePath);

    for (auto mesh : loader.LoadedMeshes_)
    {
        mesh->loadTexture(texture);
		scene->addMesh(mesh);
    }
}

void drawTriangle(Scene* scene)
{
   /* Triangle t0 = { Vec3f(-0.5, 0.5,0),   Vec3f(0.5, 0.5,0),  Vec3f(0, 0,-0) };
    Triangle t1 = { Vec3f(-0.2, 0.7,0),   Vec3f(0.6, 0.5,0),  Vec3f(-0.2, 0,-0) };
    Triangle t2 = { Vec3f(-0.1, 0.5,0),   Vec3f(0.2, 0.5,0),  Vec3f(0, 0,-0) };*/

    Triangle t0 = { {2, 0, -2},    {0, 2, -2},  {-2, 0, -2} };
    Triangle t1 = { {3.5, -1, -5},   {2.5, 1.5, -5 }, {-1, 0.5, -0} };

    t0.setColor(0, Color::Red);
    t0.setColor(1, Color::Green);
    t0.setColor(2, Color::Blue);

    t1.setColor(0, Color::Green);
    t1.setColor(1, Color::Green);
    t1.setColor(2, Color::Blue);
    

    std::vector<Geometry*>geometries;
    geometries.push_back(&t0);
    geometries.push_back(&t1);

    //scene->addModel(geometries);
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
    int key = 0;

    //cv::Mat image(scene->width_, scene->height_, CV_32FC3);
    //cv::imshow("image", image);
    //cv::setMouseCallback("image", &mouseCallBack, scene);
    bool isFirst = true;
    int frame_count = 0;
    scene->getCamera()->setMoveSpeed(-1);
    while (key != 27) {
        scene->clear();
        scene->doUpDate();

        /*auto s = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()
        );

        scene->getCamera()->setMoveSpeed(static_cast<double>(s.count()));*/
        //scene->write();
        const auto pData = scene->getDataBuffer().data();
        cv::Mat image= cv::Mat(scene->width_, scene->height_, CV_32FC3, pData);
        //cv::flip(image, image, 0);
        //std::cout << image;
        //cv::Mat image(scene->width_, scene->height_, CV_32FC3, cv::Scalar(0.2, 0.6, 0.8));
        //image.convertTo(image, CV_8UC3, 1.0f);
        //cv::Mat image(scene->width_, scene->height_, CV_8UC3, cv::Scalar(0, 0, 255));
		cv::imshow("image", image);
		cv::setMouseCallback("image", &mouseCallBack, scene);
        std::cout << scene->getCamera()->getCameraDir() << std::endl;;
        //key = cv::waitKey(10);
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

        std::cout << "frame count: " << frame_count++ << '\n';
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