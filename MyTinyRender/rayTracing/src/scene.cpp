#include <scene.h>
#include <Object.h>
#include <global.hpp>
#include <Timer.h>
#include <opencv2/highgui.hpp>

Scene::Scene(int width, int height):width_(width),height_(height)
{
	frameBuffer_.resize(width, std::vector<Vec3f>(height, 0));
	zBuffer_.resize(width, std::vector<float>(height, 0));

	//camera_ = std::make_unique<Camera>(this);
}


void Scene::render()
{
    if (way_ == Render::RenderWay::Rasterizer)
    {
        //do Rasterizer
    }
    else if (way_ == Render::RenderWay::RayTracing)
    {
        // RayTracing
        rayTracing();
    }   
}

void Scene::setRenderWay(Render::RenderWay way)
{
	way_ = way;
}


void Scene::rayTracing()
{
    Timer timer;

    auto s = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()
    );

    Vec3f eye = { 0, 0.0f,10.0f };//…„œÒª˙Œª÷√

    float imageAspectRatio = width_ / height_;
    float scale = tan(deg2rad(fov_ * 0.5));

    for (int i = 0; i < width_; i++)
    {
        for (int j = 0; j < height_; j++)
        {
            float x = 2 * (i + 0.5) / (width_ - 1) *
                imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / height_) * scale;
            Vec3f dir = Vec3f(x, y, -1).normalize();

            Ray ray(eye, dir);
            frameBuffer_[i][j] = castRay(ray);
        }
    }

    const auto pData = frameBuffer_.data();
    cv::Mat image = cv::Mat(height_, width_, CV_32FC3, pData);

    cv::imshow("image", image);

    float ms = timer.stop();
    std::cout << "time cost = " << static_cast<float>(ms) << " \n";
}

Vec3f Scene::castRay(Ray ray)
{
	return Vec3f();
}

void Scene::add(const std::shared_ptr<Object>& obj)
{
	objects_.push_back(obj);
}

void Scene::add(const std::shared_ptr<Light>& light)
{
	lights_.push_back(light);
}

int Scene::getIndex(int x, int y) const
{
	return (height_ - y - 1) * width_ + x;
}

void Scene::setDataBuffer(int x, int y, const Vec3f& data)
{
	frameBuffer_[x][y] = data;
}

void Scene::setZBuffer(int x, int y, const float val)
{
	zBuffer_[x][y] = val;
}

float Scene::getZBuffer(int x, int y) const
{
	return zBuffer_[x][y];
}