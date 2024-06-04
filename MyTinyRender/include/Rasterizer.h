#pragma once

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

private:
	Scene* scene_ = nullptr;
private:
	
};

