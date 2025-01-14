#include "scene.h"
#include "Material.h"
#include "Object.h"
#include <Timer.h>
#include <LoaderMesh.h>
#include <opencv2/highgui.hpp>
#include <atomic>
#include "BVHBuild.h"

#define USEPARALLEL


Scene::Scene(int width, int height):width_(width),height_(height)
{
	frameBuffer_.resize(width_ * height_);
	zBuffer_.resize(width_ * height_);

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

    cameraPos_ = { 278, 273, -800 };//摄像机位置

    float imageAspectRatio = width_ / height_;
    float scale = tan(GamesMath::deg2rad(fov_ * 0.5));

    //std::atomic<int> count = 0
    int count = 0;
    int spp = 1;
    std::mutex m;

#ifdef USEPARALLEL
#pragma omp parallel for
#endif // !DEBUG
    for (int i = 0; i < width_; i++)
    {
        for (int j = 0; j < height_; j++)
        {
            //从左下角开始
            float x = (2 * (i + 0.5) / width_ - 1) *
                imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / height_) * scale;
            Vec3f dir = Vec3f(-x, y, 1).normalize();

            Ray ray(cameraPos_, dir);
            Vec3f ret = castRaySPP(ray, spp);
            frameBuffer_[getIndex(i,j)] = Vec3f(ret.z, ret.y, ret.x);
        }
        std::lock_guard<std::mutex>lm(m);
        ++count;
        GamesMath::UpdateProgress(float(count) / float(width_));
    }
    GamesMath::UpdateProgress(1.0f);

    float ms = timer.stop();
    std::cout << std::endl;
    std::cout << "time cost = " << static_cast<float>(ms) << " \n";

    const auto pData = frameBuffer_.data();
    cv::Mat image = cv::Mat(height_, width_, CV_32FC3, pData);
    cv::Mat imageFlip;

    cv::flip(image, imageFlip, 0);
    cv::imshow("imageFlip", imageFlip);

    cv::waitKey(0);

    
}


Vec3f Scene::castRay(const Ray& ray, int depth)
{
    if (depth > this->maxDepth_) {
        return Vec3f(0.0, 0.0, 0.0);
    }
    Intersection intersection = intersect(ray);
    const auto& m = intersection.m;
    Object* hitObject = intersection.object;
    Vec3f hitColor = this->backgroundColor_;
    //    float tnear = kInfinity;
    Vec2f uv = intersection.st;
    uint32_t index = intersection.index;
    if (intersection.happened) {

        Vec3f hitPoint = intersection.intsCoords;
        Vec3f N = intersection.normal; // normal
        Vec2f st; // st coordinates
        hitObject->getSurfaceProperties(hitPoint, ray.dir, index, uv, N, st);
        //        Vec3f tmp = hitPoint;
        switch (m->getType())
        {
        case Material::REFLECTION_AND_REFRACTION:
        {
            Vec3f reflectionDirection = GamesMath::reflect(ray.dir, N).normalize();
            Vec3f refractionDirection = GamesMath::refract(ray.dir, N, m->ior).normalize();
            Vec3f reflectionRayOrig = (reflectionDirection.dot(N) < 0) ?
                hitPoint - N * EPSILON :
                hitPoint + N * EPSILON;
            Vec3f refractionRayOrig = (refractionDirection.dot(N) < 0) ?
                hitPoint - N * EPSILON :
                hitPoint + N * EPSILON;
            Vec3f reflectionColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1);
            Vec3f refractionColor = castRay(Ray(refractionRayOrig, refractionDirection), depth + 1);          
            float kr = GamesMath::fresnel(ray.dir, N, m->ior);
            hitColor = reflectionColor * kr + refractionColor * (1 - kr);
            break;
        }
        case Material::REFLECTION:
        {          
            float kr = GamesMath::fresnel(ray.dir, N, m->ior);
            Vec3f reflectionDirection = GamesMath::reflect(ray.dir, N);
            Vec3f reflectionRayOrig = (reflectionDirection.dot(N) < 0) ?
                hitPoint - N * EPSILON :
                hitPoint + N * EPSILON;
            hitColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1) * kr;
            break;
        }
        default:
        {
            // [comment]
            // We use the Phong illumation model int the default case. The phong model
            // is composed of a diffuse and a specular reflection component.
            // [/comment]
            Vec3f lightAmt = 0, specularColor = 0;
            Vec3f shadowPointOrig = (ray.dir.dot(N) < 0) ?
                hitPoint + N * EPSILON :
                hitPoint - N * EPSILON;
            // [comment]
            // Loop over all lights in the scene and sum their contribution up
            // We also apply the lambert cosine law
            // [/comment]
            for (uint32_t i = 0; i < lights_.size(); ++i)
            {
                {
                    Vec3f lightDir = lights_[i]->position - hitPoint;
                    Vec3f shadowPoint2lightDir = lights_[i]->position - shadowPointOrig;
                    // square of the distance between hitPoint and the light
                    float lightDistance2 = lightDir.norm2();
                    lightDir = lightDir.normalize();
                    float LdotN = std::max(0.f, lightDir.dot(N));
                    Object* shadowHitObject = nullptr;
                    float tNearShadow = FLT_MAX;
                    // is the point in shadow, and is the nearest occluding object closer to the object than the light itself?
                    // //以阴影处出发，向着光源方向发射光线，与场景中的物体求交，如果与物体相交，则说明其在物体下方，
                    //表面被物体遮住了，是在阴影中
                    //bool inShadow = bvh->Intersect(Ray(shadowPointOrig, lightDir)).happened;
                    bool inShadow = intersect(Ray(shadowPointOrig, shadowPoint2lightDir)).happened;
                    //lightAmt += (1 - inShadow) * lights_[i]->emissionColor * LdotN;
                    lightAmt += (1 - inShadow) * lights_[i]->emissionColor * LdotN;
                    Vec3f reflectionDirection = GamesMath::reflect(-lightDir, N);
                    specularColor += powf(std::max(0.f, -reflectionDirection.dot(ray.dir)),
                        m->specularExponent) * lights_[i]->emissionColor;
                }
            }
            hitColor = lightAmt * (hitObject->evalDiffuseColor(st) * m->Kd + specularColor * m->Ks);
            break;
        }
        }
    }
    if (hitColor == Vec3f(0, 0, 0))
    {
        int x = 1;
    }

    return hitColor;
}

Vec3f Scene::castRaySPP(const Ray& ray, int spp)
{
    Vec3f res;
    for (int k = 0; k < spp; k++) {
        res += pathTracing(ray) / spp;
    }
    return res;
}

Vec3f Scene::pathTracing(const Ray& ray)
{
    Vec3f lDir, lInDIr;
    Intersection intersection = intersect(ray);
    if (intersection.happened)
    {
        //lDir
        for (const auto& light: lights_)
        {
            Vec3f hitPos = intersection.intsCoords;
            Vec3f lightDir = hitPos - light->position;
            float light2Dis = lightDir.norm();
            lightDir = lightDir.normalize();
            if (intersection.m->hasEmission())
            {
                lDir = intersection.m->getEmission();
                break;
            }

            Intersection intsect = intersect(Ray(light->position, lightDir));
            if (intsect.happened && abs(intsect.distance - light2Dis) < 0.1f)
            {
                //直接光照
                Vec3f norm = intersection.normal;
                float costh1 =-lightDir.dot(norm);
                float pdf;
                sampleLight(intsect, pdf);
                float costh2 =Vec3f::dotProduct(lightDir, intsect.normal);
                Vec3f wo = intersection.m->sample(-lightDir, norm);
                lDir = light->emissionColor * intersection.m->eval(-lightDir, wo, norm) *
                    costh1 * costh2 / light2Dis / light2Dis / pdf;
            }

        }
        //间接光照
        {
            float p_RR = P_RR();
            if (p_RR != 0 && p_RR > 0.3)
            {
                Vec3f outRaydir = intersection.m->sample(ray.dir, intersection.normal);
                auto outRay = Ray(intersection.intsCoords, outRaydir);
                Intersection intsect = intersect(outRay);
                if (intsect.happened && !intsect.m->hasEmission())
                {
                    Vec3f norm = intersection.normal;
                    Vec3f hitPos = intsect.intsCoords;
                    Vec3f dir =(hitPos - cameraPos_).normalize();

                    Vec3f wo = intersection.m->sample(-outRaydir, norm);
                    float pdfHemi = 0.5 / M_PI;
                    float costh =Vec3f::dotProduct(outRaydir, norm);
                    lInDIr = castRay(Ray(cameraPos_, dir),0) * intersection.m->eval(outRaydir, wo, norm) *
                        costh / pdfHemi / p_RR;
                }
            }
        }
    }
    return lDir + lInDIr;
}






Intersection Scene::intersect(const Ray& ray)
{
    Intersection intersect;
    if (bvhNode_)
    {
        intersect = bvhNode_->intersect(ray);
    }
    else
    {
        for (uint32_t k = 0; k < objects_.size(); ++k)
        {
            const auto& ret = objects_[k]->intersect(ray);
            if (ret.distance < intersect.distance)
            {
                intersect = ret;
                intersect.object = objects_[k].get();
            }
        }
    }  
    return intersect;
}

void Scene::buildBVH()
{
    if (!bvhNode_)
    {
        bvhNode_.reset(new BVHBuild);
        bvhNode_->buildBVH(objects_);
    }

    for (uint32_t k = 0; k < objects_.size(); ++k) {
        if (objects_[k]->hasEmit()) {
            Intersection pos;
            float pdf;
            objects_[k]->sample(pos, pdf);
            lights_.push_back(std::shared_ptr<Light>(new Light(pos.intsCoords, objects_[k]->m->m_emission)));
        }
    }
}

void Scene::add(const std::shared_ptr<Object>& obj)
{
	objects_.push_back(obj);
}

void Scene::add(const std::shared_ptr<Light>& light)
{
	lights_.push_back(light);
}

void Scene::sampleLight(Intersection& pos, float& pdf) const
{
    float emitAreaSum = 0;
    for (uint32_t k = 0; k < objects_.size(); ++k) {
        if (objects_[k]->hasEmit()) {
            emitAreaSum += objects_[k]->getArea();
        }
    }
    float p = GamesMath::getRandomFloat() * emitAreaSum;
    emitAreaSum = 0;
    for (uint32_t k = 0; k < objects_.size(); ++k) {
        if (objects_[k]->hasEmit()) {
            emitAreaSum += objects_[k]->getArea();
            if (p <= emitAreaSum) {
                objects_[k]->sample(pos, pdf);
                break;
            }
        }
    }
}

float Scene::P_RR() const
{
    float q = GamesMath::getRandomFloat();
    float p = GamesMath::getRandomFloat();
    if (p > q)
    {
        return p;
    }
    return 0;
}

int Scene::getIndex(int x, int y) const
{
	return (height_ - y - 1) * width_ + x;
}

void Scene::setDataBuffer(int x, int y, const Vec3f& data)
{
    frameBuffer_[getIndex(x, y)] = data;
}

void Scene::setZBuffer(int x, int y, const float val)
{
	zBuffer_[getIndex(x,y)] = val;
}

float Scene::getZBuffer(int x, int y) const
{
	return zBuffer_[getIndex(x, y)];
}