#include "Rasterizer.h"
#include "Geometry.h"
#include "Mesh.h"
#include "Scene.h"
#include "Shader.h"


Rasterizer::Rasterizer(Scene* scene) :scene_(scene)
{
}

void Rasterizer::rasterizeTriangle(int i, int j, Triangle* triangle, const Mesh* mesh)
{
	/*const Vec2i pixel{ i ,j };
	const Vec2f pixelf{ i + 0.5f,j + 0.5f };*/

	constexpr RasterizerWay way = RasterizerWay::BARYCENTRIC;

	if (way == RasterizerWay::BARYCENTRIC)
	{
		if (!triangle->inBoundBox(i + 0.5f, j + 0.5f))
			return;

		rasterizeTriangle(i, j, way, triangle, mesh);
	}
	else if (way == RasterizerWay::MASS)
	{
		if (!triangle->inNearBoundBox(i + 0.5f, j + 0.5f, 1.0f))
			return;;

		rasterizeTriangle(i, j, way, triangle, mesh);
	}

}

void Rasterizer::rasterizeTriangle(int i, int j, RasterizerWay way, Triangle* triangle, const Mesh* mesh)
{
	const Vec2f pixel{ i + 0.5f,j + 0.5f };
	if (way == RasterizerWay::BARYCENTRIC || way == RasterizerWay::DEFAULT)
	{
		if (insideTriangle(pixel.x, pixel.y, triangle->vertexs()))
		{
			switch (way)
			{
			case RasterizerWay::DEFAULT:
			{
				///使用深度图
				///
				if (triangle->vertexs()->Z() < scene_->getZBuffer(i, j))
				{
					scene_->setDataBuffer(i, j, triangle->getColor());
					scene_->setZBuffer(i, j, triangle->vertexs()->Z());
				}
				break;
			}
			case RasterizerWay::BARYCENTRIC:
			{

				///使用重心坐标
				auto [alpha, beta, gamma] = computeBarycentric2D(pixel.x, pixel.y, triangle->vertexs());
				const float& weight = alpha + beta + gamma;
				//const float& pixelZ = (alpha * triangle->vertexs(0).z + beta * triangle->vertexs(1).z + gamma * triangle->vertexs(2).z) / weight;
				//const Vec3f& color = alpha * triangle->getColor(0) + beta * triangle->getColor(1) + gamma * triangle->getColor(2);
				//const Vec2f& coordTexture = alpha * triangle->getTexCoord(0) + beta * triangle->getTexCoord(1) + gamma * triangle->getTexCoord(2);
				//const Vec2f& coordTexture = alpha * triangle->getNormal(0) + beta * triangle->getTexCoord(1) + gamma * triangle->getTexCoord(2);
				const float& pixelZ = interpolate(alpha, beta, gamma, triangle->vertexs(0).z, triangle->vertexs(0).z, triangle->vertexs(0).z, weight);
				const Vec2f& coordTexture = interpolate(alpha, beta, gamma, triangle->getTexCoord(0), triangle->getTexCoord(1), triangle->getTexCoord(2), weight);
				const Vec3f& color = interpolate(alpha, beta, gamma, triangle->getColor(0), triangle->getColor(1), triangle->getColor(2), weight);
				const Vec3f& normal = interpolate(alpha, beta, gamma, triangle->getNormal(0), triangle->getNormal(1), triangle->getNormal(2), weight);

				fragment_shader_payload fsp(color, normal, coordTexture, mesh);
				///到此为止都可以有CPU执行，下面部分则可以尝试使用GPU执行
				if (pixelZ < scene_->getZBuffer(i, j))
				{
					Vec3f dataBuffer;
					if (mesh && mesh->way_ == Mesh::USE_TEXTURE)
					{
						Vec3f textureColor;
						for (const auto& item : mesh->textures_)
						{
							textureColor += item->getColorBilinear(coordTexture);
						}

						dataBuffer = textureColor / 255.f;
					}
					else
					{
						dataBuffer = color;
					}
					// to frameShaderFun
					scene_->setDataBuffer(i, j, fragmentShaderFunction_(fsp));
					//TO be BGR
					//scene_->setDataBuffer(i, j, { dataBuffer.z,dataBuffer.y,dataBuffer.x });
					//setDataBuffer(i, j, color);
					scene_->setZBuffer(i, j, pixelZ);
				}
				break;
			}
			}

		}
	}
	else if (way == RasterizerWay::MASS)
	{
		Vec3f colorVal;
		int insideCount = 0;
		for (int m = 0; m < scene_->msaaH_; ++m)
		{
			for (int n = 0; n < scene_->msaaV_; ++n)
			{
				float p = (0.5f + m) / scene_->msaaH_;
				float q = (0.5f + n) / scene_->msaaV_;
				Vec2f msaaPixel = { i + p,j + q };
				if (!triangle->inBoundBox(msaaPixel))
					continue;
				if (insideTriangle(msaaPixel, triangle->vertexs()))
				{
					auto [alpha, beta, gamma] = computeBarycentric2D(msaaPixel.x, msaaPixel.y, triangle->vertexs());
					const Vec3f& color = alpha * triangle->getColor(0) + beta * triangle->getColor(1) + gamma * triangle->getColor(2);
					const float& pixelZ = alpha * triangle->vertexs(0).z + beta * triangle->vertexs(1).z + gamma * triangle->vertexs(2).z;
					if (pixelZ <= scene_->getMassZBuffer(i, j, m, n))
					{
						scene_->setMassZBuffer(i, j, m, n, pixelZ);
						colorVal += color;
						++insideCount;
					}
				}
			}
		}
		colorVal = colorVal / scene_->msaaSpp_;
		colorVal = mixture(colorVal, scene_->getDataBuffer(i, j), 1.0f * insideCount / scene_->msaaSpp_);
		scene_->setDataBuffer(i, j, colorVal);
	}
}

template<typename T>
T Rasterizer::interpolate(float alpha, float beta, float gamma, const T& vert1, const T& vert2,
	const T& vert3, float weight)
{
	return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}
