#pragma once
#include "Vector.hpp"

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
    float Kd, Ks;
    float specularExponent;
    //Texture tex;

    inline Material(MaterialType t = DIFFUSE_AND_GLOSSY, Vec3f c = Vec3f(1, 1, 1), Vec3f e = Vec3f(0, 0, 0));
    inline MaterialType getType();
    inline Vec3f getColor();
    inline Vec3f getColorAt(double u, double v);
    inline Vec3f getEmission();
};

Material::Material(MaterialType t, Vec3f c, Vec3f e) {
    m_type = t;
    m_color = c;
    m_emission = e;
}

Material::MaterialType Material::getType() { return m_type; }
Vec3f Material::getColor() { return m_color; }
Vec3f Material::getEmission() { return m_emission; }

Vec3f Material::getColorAt(double u, double v) {
    return Vec3f();
}
