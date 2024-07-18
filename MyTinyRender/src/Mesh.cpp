#include <Mesh.h>
#include <Geometry.h>


Mesh::Mesh(const std::vector<Vertex>& vertices) :vertices_(vertices)
{
    modelMatrix_.identity();
}

Mesh::Mesh(const std::vector<Triangle*>& Triangles)
{
    for (const auto& t : Triangles)
    {
        addTriangle(t);
    }

    modelMatrix_.identity();
}

void Mesh::addTriangle(const Triangle* t)
{
    for (int i = 0; i < 3; ++i)
    {
        Vertex v;
        v.position_ = t->vertexs(i);
        v.normal_ = t->getNormal(i);
        v.textureCoordinate_ = t->getTexCoord(i);
        vertices_.push_back(v);
    }
}
