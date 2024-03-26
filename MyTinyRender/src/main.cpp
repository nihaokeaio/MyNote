# pragma warning (disable:4819)
#include "Vector.hpp"
#include "Scene.h"
#include "global.hpp"
#include "Geometry.h"

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
    scene.useMsaa(2, 2);

    drawTriangle(&scene);
	
    scene.write();
	return 0;
}