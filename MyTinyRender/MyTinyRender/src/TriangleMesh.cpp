#include "TriangleMesh.h"
#include <Geometry.h>


TriangleMesh::TriangleMesh(const std::vector<Triangle*>& Triangles)
{
    for (const auto& t : Triangles)
    {
        addTriangle(t);
    }

    modelMatrix_.identity();
}

void TriangleMesh::addTriangle(const Triangle* t)
{
    for (int i = 0; i < 3; ++i)
    {
        LoaderMeshSpace::Vertex v;
        v.position_ = t->vertexs(i);
        v.normal_ = t->getNormal(i);
        v.textureCoordinate_ = t->getTexCoord(i);
        v.color_ = t->getColor(i);
        vertices_.push_back(v);
    }
}

void TriangleMesh::setModelMat(float rotateAngle, float scale, const Vec3f& pos)
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
