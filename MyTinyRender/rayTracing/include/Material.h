#pragma once
#include "Vector.hpp"
#include "global.hpp"

class Material 
{
public:
    enum MaterialType 
    {
        DIFFUSE_AND_GLOSSY,
        REFLECTION_AND_REFRACTION,
        REFLECTION 
    };

    MaterialType m_type;
    Vec3f m_color;
    Vec3f m_emission;
    float ior;
    float Ka = 0.1;
    float Kd = 0.7;
    float Ks=0.2;
    float specularExponent=25.0;
    //Texture tex;

    inline Material(MaterialType t = DIFFUSE_AND_GLOSSY, Vec3f c = Vec3f(1, 1, 1), Vec3f e = Vec3f(0, 0, 0));
    inline MaterialType getType();
    inline Vec3f getColor();
    inline Vec3f getColorAt(double u, double v);
    inline Vec3f getEmission();
    inline bool hasEmission();

    inline Vec3f sample(const Vec3f& wi, const Vec3f& N);
    inline float pdf(const Vec3f& wi, const Vec3f& wo, const Vec3f& N);
    inline Vec3f eval(const Vec3f& wi, const Vec3f& wo, const Vec3f& N);
};

Material::Material(MaterialType t, Vec3f c, Vec3f e) {
    m_type = t;
    m_color = c;
    m_emission = e;
}

Material::MaterialType Material::getType() { return m_type; }
Vec3f Material::getColor() { return m_color; }
Vec3f Material::getEmission() { return m_emission; }

inline bool Material::hasEmission()
{
    return m_emission.norm2()> EPSILON;
}

Vec3f Material::getColorAt(double u, double v) 
{
    return Vec3f();
}

Vec3f Material::sample(const Vec3f& wi, const Vec3f& N)
{
    switch (m_type) {
    case DIFFUSE_AND_GLOSSY:
    {
        // uniform sample on the hemisphere
        float x_1 =GamesMath::get_random_float(), x_2 = GamesMath::get_random_float();
        float z = std::fabs(1.0f - 2.0f * x_1);
        float r = std::sqrt(1.0f - z * z), phi = 2 * M_PI * x_2;
        Vec3f localRay(r * std::cos(phi), r * std::sin(phi), z);
        return GamesMath::toWorld(localRay, N);
    }
    }
}

float Material::pdf(const Vec3f& wi, const Vec3f& wo, const Vec3f& N) 
{
    switch (m_type)
    {
    case DIFFUSE_AND_GLOSSY:
    {
        // uniform sample probability 1 / (2 * PI)
        if (Vec3f::dotProduct(wo, N) > 0.0f)
            return 0.5f / M_PI;
        else
            return 0.0f;
        break;
    }
    }
}

Vec3f Material::eval(const Vec3f& wi, const Vec3f& wo, const Vec3f& N) 
{
    switch (m_type) {
    case DIFFUSE_AND_GLOSSY:
    {
        // calculate the contribution of diffuse   model
        float cosalpha = Vec3f::dotProduct(N, wo);
        if (cosalpha > 0.0f) {
            Vec3f diffuse = Kd / M_PI;
            return diffuse;
        }
        else
            return Vec3f(0.0f);
        break;
    }
    }
}
