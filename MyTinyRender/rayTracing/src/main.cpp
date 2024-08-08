#include <iostream>
#include "Triangle.h"
#include "scene.h"
#include "Render.h"

int main()
{
	Scene scene(800, 800);

    std::shared_ptr<TriangleMesh> bunny=std::make_shared<TriangleMesh>("../models/bunny/bunny.obj");
    const auto& light1 = std::make_shared<Light>(Vec3f(-20, 70, 20), 1);
    const auto& light2 = std::make_shared<Light>(Vec3f(20, 70, 20), 1);

    scene.add(bunny);
    scene.add(light1);
    scene.add(light2);
    //scene.buildBVH();

    Render r;

    auto start = std::chrono::system_clock::now();
    r.addScene(&scene, Render::RenderWay::RayTracing);
    r.render();
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
	
}