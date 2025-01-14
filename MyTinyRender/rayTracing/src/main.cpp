#include <iostream>
#include "Triangle.h"
#include "sphere.h"
#include "scene.h"
#include "Render.h"
#include "Material.h"



int main()
{
	Scene scene(1280, 960);

    std::shared_ptr<Material> red(new Material(Material::DIFFUSE_AND_GLOSSY, Vec3f(0.0f)));
    red->Kd = Vec3f(0.63f, 0.065f, 0.05f);
    std::shared_ptr<Material> green(new Material(Material::DIFFUSE_AND_GLOSSY, Vec3f(0.0f)));
    green->Kd = Vec3f(0.14f, 0.45f, 0.091f);
    std::shared_ptr<Material> white(new Material(Material::DIFFUSE_AND_GLOSSY, Vec3f(0.0f)));
    white->Kd = Vec3f(0.725f, 0.71f, 0.68f);
    std::shared_ptr<Material> lightM(new Material(Material::DIFFUSE_AND_GLOSSY, 
        (8.0f * Vec3f(0.747f + 0.058f, 0.747f + 0.258f, 0.747f) + 
            15.6f * Vec3f(0.740f + 0.287f, 0.740f + 0.160f, 0.740f) + 
            18.4f * Vec3f(0.737f + 0.642f, 0.737f + 0.159f, 0.737f))));
    lightM->Kd = Vec3f(0.65f);

    std::shared_ptr<TriangleMesh> floor(new TriangleMesh("./Resource/models/cornellbox/floor.obj", white));
    std::shared_ptr<TriangleMesh> shortbox(new TriangleMesh("./Resource/models/cornellbox/shortbox.obj", white));
    std::shared_ptr<TriangleMesh> tallbox(new TriangleMesh("./Resource/models/cornellbox/tallbox.obj", white));
    std::shared_ptr<TriangleMesh> left(new TriangleMesh("./Resource/models/cornellbox/left.obj", red));
    std::shared_ptr<TriangleMesh> right(new TriangleMesh("./Resource/models/cornellbox/right.obj", green));
    std::shared_ptr<TriangleMesh> light(new TriangleMesh("./Resource/models/cornellbox/light.obj", lightM));
    std::shared_ptr<TriangleMesh> bunny(new TriangleMesh("./Resource/models/bunny/bunny.obj", white));

    bunny->setScale(1000.0);
    bunny->setMove(Vec3f(250));

    scene.add(floor);
    //scene.add(shortbox);
    //scene.add(tallbox);
    scene.add(left);
    scene.add(bunny);
    scene.add(right);
    scene.add(light);
    scene.buildBVH();

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