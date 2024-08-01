#pragma once

#include "Vector.hpp"
#include  "Texture.h"

class Triangle;

struct Vertex
{
	Vec3f position_;

	Vec3f normal_;

    Vec3f color_ = Color::Gray;

	Vec2f textureCoordinate_;
};

struct Material
{
    Material()
    {
        name_="";
        Ns = 0.0f;
        Ni = 0.0f;
        d = 0.0f;
        illum = 0;
    }

    // Material Name
    std::string name_;
    // Ambient Color
    Vec3f Ka;
    // Diffuse Color
    Vec3f Kd;
    // Specular Color
    Vec3f Ks;
    // Specular Exponent
    float Ns;
    // Optical Density
    float Ni;
    // Dissolve
    float d;
    // Illumination
    int illum;
    // Ambient Texture Map
    std::string map_Ka;
    // Diffuse Texture Map
    std::string map_Kd;
    // Specular Texture Map
    std::string map_Ks;
    // Specular Hightlight Map
    std::string map_Ns;
    // Alpha Texture Map
    std::string map_d;
    // Bump Map
    std::string map_bump;
};



struct Mesh
{
    enum  RenderWay
    {
        USE_COLOR       = 0,
        USE_TEXTURE     = 1
	};
    Mesh() = default;

    Mesh(const std::vector<Vertex>& vertices);

    Mesh(const std::vector<Triangle*>& Triangles);
    

    void addTriangle(const Triangle* t);
    

    Mesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices):vertices_(vertices),indices_(indices)
    {
        modelMatrix_.identity();
    }

    ~Mesh()
    {
	    
    }



    void loadTexture(const std::shared_ptr<Texture>& texture)
    {
	    textures_.emplace_back(texture);
        way_ = USE_COLOR;
    }

    void setRenderWay(RenderWay way) { way_ = way; }

    void setModelMat(const Matrix4x4f& mat) { modelMatrix_ = mat; }
    void setModelMat(float rotateAngle = 0.f, float scale = 1.0f, const Vec3f& pos = (0, 0, 0));

    std::string name_;
    std::vector<Vertex> vertices_;
    std::vector<unsigned int>indices_;
    std::vector<std::shared_ptr<Texture>> textures_;

    Matrix4x4f modelMatrix_;

    Material material;

    RenderWay way_ = USE_TEXTURE;
};

inline void Mesh::setModelMat(float rotateAngle, float scale, const Vec3f& pos)
{
    Matrix4x4f rotationMat;
    const auto angle = rotateAngle * M_PI / 180.f;
    rotationMat = Matrix4x4f{
        cos(angle), 0, sin(angle), 0,
        0, 1, 0, 0,
        -sin(angle), 0, cos(angle), 0,
        0, 0, 0, 1 };

    Matrix4x4f scaleMat;
    scaleMat = Matrix4x4f{
        scale, 0, 0, 0,
        0, scale, 0, 0,
        0, 0, scale, 0,
        0, 0, 0, 1 };

    Matrix4x4f translateMat;
    translateMat = Matrix4x4f{
        1, 0, 0, pos.x,
        0, 1, 0, pos.y,
        0, 0, 1, pos.z,
        0, 0, 0, 1 };

    modelMatrix_ = translateMat * rotationMat * scaleMat;
}
