# pragma warning (disable:4819)
#include "Vector.hpp"
#include "Scene.h"
#include "global.hpp"
#include "Geometry.h"

void triangle0(Vec3f* V, Scene& scene, Vec3f* colors)
{
    Vec2f bboxmin = Vec2f(scene.width_ - 1, scene.height_ - 1);
    Vec2f bboxmax = Vec2f();
    Vec2f clamp(scene.width_ - 1, scene.height_ - 1);
    for (int i = 0; i < 3; ++i)
    {
        bboxmin.x = std::max(0.0f, std::min(bboxmin.x, V[i].x));
        bboxmin.y = std::max(0.0f, std::min(bboxmin.y, V[i].y));

        bboxmax.x = std::min(clamp.x, std::max(bboxmax.x, V[i].x));
        bboxmax.y = std::min(clamp.y, std::max(bboxmax.y, V[i].y));
    }

    for (int x = bboxmin.x; x <= bboxmax.x; ++x)
    {
        for (int y = bboxmin.y; y <= bboxmax.y; ++y)
        {
            auto [alpha, beta, gamma] = computeBarycentric2D(x + 0.5f, y +0.5f, V);
            if (alpha < 0 || beta < 0 || gamma < 0)continue;
            Vec3f color = alpha * colors[0] + beta * colors[1] + gamma * colors[2];
            float z_deep= alpha * V[0].z + beta * V[1].z + gamma * V[2].z;
            if (scene.get_z_buffer(x, y) > z_deep)
            {
                scene.set_z_buffer(x, y, z_deep);
                scene.set_data_buffer(x, y, color);
            }          
        }
    }
}

void drawTriangle(Scene* scene)
{
   /* Triangle t0 = { Vec3f(-0.5, 0.5,0),   Vec3f(0.5, 0.5,0),  Vec3f(0, 0,-0) };
    Triangle t1 = { Vec3f(-0.2, 0.7,0),   Vec3f(0.6, 0.5,0),  Vec3f(-0.2, 0,-0) };
    Triangle t2 = { Vec3f(-0.1, 0.5,0),   Vec3f(0.2, 0.5,0),  Vec3f(0, 0,-0) };*/

    Triangle t0 = { {2, 0, -2},    {0, 2, -2},  {-2, 0, -2} };
    Triangle t1 = { {3.5, -1, -5},   {2.5, 1.5, -5 }, {-1, 0.5, -5} };

    t0.setColor(0, Color::Red);
    t0.setColor(1, Color::Green);
    t0.setColor(2, Color::Blue);

    t1.setColor(0, Color::Green);
    t1.setColor(1, Color::Green);
    t1.setColor(2, Color::Blue);
    

    std::vector<Geometry*>geometries;
    geometries.push_back(&t0);
    geometries.push_back(&t1);
    //geometries.push_back(&t2);

    scene->addModel(geometries);
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

    scene->addModel(geometries);
}

int main()
{
    Scene scene;

    drawTriangle(&scene);
	
    scene.write();
	return 0;
}