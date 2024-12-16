#include <iostream>
#include "Triangle.h"
#include "sphere.h"
#include "scene.h"
#include "Render.h"
#include "Material.h"



int main()
{
	Scene scene(1280, 960);

    std::shared_ptr<TriangleMesh> bunny=std::make_shared<TriangleMesh>("./Resource/models/bunny/bunny.obj");
    const auto& light1 = std::make_shared<Light>(Vec3f(-20, 70, 20), 1);
    const auto& light2 = std::make_shared<Light>(Vec3f(20, 70, 20), 1);

    float x = 15;
    float y = -0;
    float z = 15;
    auto material = std::make_shared<Material>(Material::MaterialType::DIFFUSE_AND_GLOSSY,
        Vec3f(0.5, 0.5, 0.5), Vec3f(0, 0, 0));
    material->Kd = 0.6;
    material->Ks = 0.0;
    material->specularExponent = 32;
    std::shared_ptr<Triangle> t0(new Triangle{ {-x, y, -z},   {-x, y, z},  {x, y, z},material });
    std::shared_ptr<Triangle> t1(new Triangle{ {x, y, -z},   {-x, y, -z }, {x, y, z},material });


    auto sph1m = std::make_shared<Material>(Material::MaterialType::REFLECTION_AND_REFRACTION,
        Vec3f(0.4, 0.4, 0.8), Vec3f(0, 0, 0));
    sph1m->ior = 9.3;
    auto sph1 = std::make_shared<Sphere>(Vec3f(-1, 5, -3), 2, sph1m.get());

    auto sph2m = std::make_shared<Material>(Material::MaterialType::REFLECTION_AND_REFRACTION,
        Vec3f(0.4, 0.4, 0.8), Vec3f(0, 0, 0));
    sph2m->ior = 1.5;
    auto sph2 = std::make_shared<Sphere>(Vec3f(3.5, 3.5, -3), 1.5, sph2m.get());


    Vec3Vector verts = { {-5,-3,-6}, {5,-3,-6}, {5,-3,-16}, {-5,-3,-16} };
    std::vector<uint> vertIndex = { 0, 1, 3, 2, 3, 1 };
    std::vector<Vec2f> st= { {0, 0}, {1, 0}, {1, 1}, {0, 1} };
    auto mesh = std::make_shared<TriangleMesh>(verts, vertIndex, st);
    //mesh->m->m_type = Material::MaterialType::DIFFUSE_AND_GLOSSY;

    scene.add(mesh);
    
    /*scene.add(t0);
    scene.add(t1);*/

    scene.add(sph1);
    scene.add(sph2);

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