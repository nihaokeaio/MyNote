#pragma once
#include "LoaderMesh.h"
#include "Vector.hpp"
#include  "Texture.h"

class Triangle;

class TriangleMesh :public Mesh
{
public:
    enum  RenderWay
    {
        USE_COLOR = 0,
        USE_TEXTURE = 1
    };
    TriangleMesh() = default;

    TriangleMesh(Mesh* m)
    {
        vertices_ = m->vertices_;
        indices_ = m->indices_;
        name_ = m->name_;
        material_ = m->material_;
    }

    TriangleMesh(const std::vector<Triangle*>& Triangles);

    void addTriangle(const Triangle* t);


    TriangleMesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices) :Mesh(vertices, indices)
    {
        modelMatrix_.identity();
    }

    ~TriangleMesh()
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

    std::vector<std::shared_ptr<Texture>> textures_;
    Matrix4x4f modelMatrix_;
    RenderWay way_ = USE_TEXTURE;
};