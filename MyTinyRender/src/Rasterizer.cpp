#include "Rasterizer.h"
#include "Geometry.h"
#include "Mesh.h"
#include "Scene.h"


Rasterizer::Rasterizer(Scene* scene) :scene_(scene)
{
}

void Rasterizer::rasterizeTriangle(int i, int j, Triangle* triangle, const Mesh* mesh)
{
	const Vec2i pixel{ i ,j };
	const Vec2f pixelf{ i + 0.5f,j + 0.5f };

	constexpr RasterizerWay way = RasterizerWay::BARYCENTRIC;

	if (way == RasterizerWay::BARYCENTRIC)
	{
		if (!triangle->inBoundBox(pixelf))
			return;

		rasterizeTriangle(pixel.x, pixel.y, way, triangle, mesh);
	}
	else if (way == RasterizerWay::MASS)
	{
		if (!triangle->inNearBoundBox(pixelf, 1.0f))
			return;;

		rasterizeTriangle(pixel.x, pixel.y, way, triangle, mesh);
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
				float weight = alpha + beta + gamma;
				const float& pixelZ = alpha * triangle->vertexs(0).z + beta * triangle->vertexs(1).z + gamma * triangle->vertexs(2).z;
				const Vec3f& color = alpha * triangle->getColor(0) + beta * triangle->getColor(1) + gamma * triangle->getColor(2);
				const Vec2f& coordTexture = alpha * triangle->getTexCoord(0) + beta * triangle->getTexCoord(1) + gamma * triangle->getTexCoord(2);
				///到此为止都可以有CPU执行，下面部分则可以尝试使用GPU执行
				if (pixelZ <= scene_->getZBuffer(i, j))
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

					scene_->setDataBuffer(i, j, dataBuffer );
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
