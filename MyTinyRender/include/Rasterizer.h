#pragma once
#include "Vector.hpp"

struct fragment_shader_payload;
struct vertex_shader_payload;
class Scene;
class Triangle;
struct Mesh;

class Rasterizer
{
	enum RasterizerWay
	{
		DEFAULT = 0,
		BARYCENTRIC = 1,
		MASS = 2
	};

public:
	explicit Rasterizer(Scene* scene);

	void rasterizeTriangle(int i, int y, Triangle* triangle, const Mesh*  mesh = nullptr);
	void rasterizeTriangle(int i, int y, RasterizerWay way, Triangle* triangle, const Mesh* mesh);

	void setFragmentShaderFunction(const std::function<Vec3f(fragment_shader_payload)>& fun) { fragmentShaderFunction_ = fun; }
	void setVertexShaderFunction(const std::function<Vec3f(vertex_shader_payload)>& fun) { vertexShaderFunction_ = fun; }

	template <typename T>
	static T interpolate(float alpha, float beta, float gamma, const T& vert1, const T& vert2, const T& vert3, float weight);
private:
	Scene* scene_ = nullptr;

	std::function<Vec3f(fragment_shader_payload)>fragmentShaderFunction_;
	std::function<Vec3f(vertex_shader_payload)>vertexShaderFunction_;
private:
	
};

