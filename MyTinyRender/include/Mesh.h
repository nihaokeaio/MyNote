#pragma once

#include "Vector.hpp"
#include  "Texture.h"


struct Vertex
{
	Vec3f position_;

	Vec3f normal_;

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

    Mesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices):vertices_(vertices),indices_(indices)
    {
        modelMatrix_.setIdentity();
    }

    void loadTexture(const std::shared_ptr<Texture>& texture) { textures_.emplace_back(texture); }

    void setRenderWay(RenderWay way) { way_ = way; }


    std::string name_;
    std::vector<Vertex> vertices_;
    std::vector<unsigned int>indices_;
    std::vector<std::shared_ptr<Texture>> textures_;

    Eigen::Matrix4f modelMatrix_;

    Material material;

    RenderWay way_ = USE_TEXTURE;
};
