#include "Scene.h"
#include "global.hpp"
#include "Geometry.h"

#include <Eigen/Eigen>

Scene::Scene()
{
	dataBuffer_.resize(width_ * height_);
	zBuffer_.resize(width_ * height_);
	std::fill(zBuffer_.begin(), zBuffer_.end(), std::numeric_limits<float>::infinity());
}

void Scene::addModel(const std::vector<Geometry*> geometries)
{
	for(const auto& model:geometries)
	{
		Triangle* triangle = dynamic_cast<Triangle*>(model);
		if (!triangle)
			return;
		//投影变换

		//视口变化
		Eigen::Matrix4f modelMatrix = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f viewMatrix = setViewMatrix(Vec3f(0, 0, 5.f));
		Eigen::Matrix4f pojMatrix = projectMatrix();


		Eigen::Matrix4f mvp = pojMatrix * viewMatrix * modelMatrix;

		std::vector<Eigen::Vector4f> vec = triangle->setModelMatrix(mvp);
		viewportMatrix(vec);

		//光栅化
		Triangle t;
		for(int i=0;i<3;++i)
		{
			Vec3f v = { vec[i].x(),vec[i].y(),vec[i].z() };
			t.setVertex(i, v);
		}
		t.boxConstruct();
		t.setColors(triangle->getColors());
		rasterizeTriangle(&t);
	}
}

void Scene::rasterizeTriangle(Geometry* geometry)
{
	Triangle* triangle = dynamic_cast<Triangle*>(geometry);
	if(!triangle)
		return;

	RasterizerWay way = RasterizerWay::MASS;
	for(int i=0;i< width_;i++)
	{
		for(int j=0;j<height_;j++)
		{
			
			const Vec2f pixel{ i + 0.5f,j + 0.5f };
			if(way==RasterizerWay::BARYCENTRIC)
			{
				if (!triangle->inBoundBox(pixel))
					
				rasterizeTriangle(pixel.x, pixel.y, way, triangle);
			}
			else if (way == RasterizerWay::MASS)
			{
				if (!triangle->inNearBoundBox(pixel, 1.0f))
					continue;

				rasterizeTriangle(pixel.x, pixel.y, way, triangle);
			}
			
		}
	}
}

void Scene::rasterizeTriangle(int i, int j, RasterizerWay way, Triangle* triangle)
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
				if (triangle->vertexs()->Z() < getZBuffer(i, j))
				{
					setDataBuffer(i, j, triangle->getColor());
					setZBuffer(i, j, triangle->vertexs()->Z());
				}
				break;
			}
			case RasterizerWay::BARYCENTRIC:
			{
				///使用重心坐标
				auto [alpha, beta, gamma] = computeBarycentric2D(pixel.x, pixel.y, triangle->vertexs());
				const Vec3f& color = alpha * triangle->getColor(0) + beta * triangle->getColor(1) + gamma * triangle->getColor(2);
				const float& pixelZ = alpha * triangle->vertexs(0).z + beta * triangle->vertexs(1).z + gamma * triangle->vertexs(2).z;
				if (pixelZ <= getZBuffer(i, j))
				{
					setDataBuffer(i, j, color);
					setZBuffer(i, j, pixelZ);
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
		for (int m = 0; m < msaaH_; ++m)
		{
			for (int n = 0; n < msaaV_; ++n)
			{
				float p = (0.5f + m) / msaaH_;
				float q = (0.5f + n) / msaaV_;
				Vec2f msaaPixel = { i + p,j + q };
				if (!triangle->inBoundBox(msaaPixel))
					continue;
				if (insideTriangle(msaaPixel, triangle->vertexs()))
				{
					auto [alpha, beta, gamma] = computeBarycentric2D(msaaPixel.x, msaaPixel.y, triangle->vertexs());
					const Vec3f& color = alpha * triangle->getColor(0) + beta * triangle->getColor(1) + gamma * triangle->getColor(2);
					const float& pixelZ = alpha * triangle->vertexs(0).z + beta * triangle->vertexs(1).z + gamma * triangle->vertexs(2).z;
					if (pixelZ <= getMassZBuffer(i, j, m, n))
					{
						setMassZBuffer(i, j, m, n, pixelZ);
						colorVal += color;
						++insideCount;
					}
				}
			}
		}
		colorVal = colorVal / msaaSpp_;
		colorVal = mixture(colorVal, getDataBuffer(i, j), 1.0f * insideCount / msaaSpp_);
		setDataBuffer(i, j, colorVal);
	}
}

Eigen::Matrix4f Scene::projectMatrix() const
{
	Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
	float zNear = near;
	float zFar = far;
	projection << zNear, 0, 0, 0,
		0, zNear, 0, 0,
		0, 0, zNear + zFar, -zNear * zFar,
		0, 0, 1, 0;
	Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
	float H = 2 * -zNear * tan(eyeFov_ / 180 * M_PI / 2);
	float W =aspectRatio_ * H;

	Eigen::Matrix4f ortho = Eigen::Matrix4f::Identity();

	ortho << 2 / W, 0, 0, 0,
		0, 2 / H, 0, 0,
		0, 0, 2 / (zNear - zFar), 0,
		0, 0, 0, 1;

	projection = scale * ortho * projection;

	return projection;
}

void  Scene::viewportMatrix(std::vector<Eigen::Vector4f>& vec)
{
	float f1 = (far + near) / 2.0f;
	float f2 = (far - near) / 2.0f;

	for(auto& v: vec)
	{
		v /= v.w();
	}

	for (auto& v : vec)
	{
		v.x() = 0.5f * width_ * (v.x() + 1.0f);
		v.y() = 0.5f * height_ * (v.y() + 1.0f);
		v.z() = v.z() * f2 + f1;
	}
}

Eigen::Matrix4f Scene::setViewMatrix(Vec3f eyePos)
{
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

	Eigen::Matrix4f translate;
	translate << 1, 0, 0, -eyePos[0],
		0, 1, 0, -eyePos[1],
		0, 0, 1, -eyePos[2],
		0, 0, 0, 1;

	view = translate * view;

	return view;
}

int Scene::getIndex(int x, int y) const
{
	return (height_ - y - 1) * width_ + x;
}

int Scene::getMsaaIndex(int h, int v) const
{
	return v + h * msaaV_;
}

Vec3f Scene::getDataBuffer(int index) const
{
	return dataBuffer_[index];
}

Vec3f Scene::getDataBuffer(int x, int y) const
{
	return dataBuffer_[getIndex(x, y)];
}

void Scene::setDataBuffer(int x, int y, const Vec3f& data)
{
	dataBuffer_[getIndex(x, y)] = data;
}

void Scene::setDataBuffer(int index, const Vec3f& data)
{
	dataBuffer_[index] = data;
}

void Scene::setZBuffer(int index, const float val)
{
	zBuffer_[index] = val;
}

void Scene::setZBuffer(int x, int y, const float val)
{
	zBuffer_[getIndex(x, y)] = val;
}

float Scene::getZBuffer(int index) const 
{
	return zBuffer_[index];
}

float Scene::getZBuffer(int x, int y) const 
{
	return zBuffer_[getIndex(x, y)];
}

void Scene::setMassZBuffer(int x, int y, int h, int v, const float val)
{
	zBufferMsaa_[getIndex(x, y)][getMsaaIndex(h, v)] = val;
}

void Scene::setMassZBuffer(int index, int h, int v, const float val)
{
	zBufferMsaa_[index][getMsaaIndex(h, v)] = val;
}

float Scene::getMassZBuffer(int x, int y, int h, int v) const
{
	return zBufferMsaa_[getIndex(x, y)][getMsaaIndex(h, v)];
}

float Scene::getMassZBuffer(int index, int h, int v) const
{
	return zBufferMsaa_[index][getMsaaIndex(h, v)];
}

void Scene::write() const
{
	FILE* fp = fopen("./output/binary.ppm", "wb");
	(void)fprintf(fp, "P6\n%d %d\n255\n", width_, height_);
	for (auto i = 0; i < height_ * width_; ++i) {
		static unsigned char color[3];
		color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, dataBuffer_[i].x), 0.6f));
		color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, dataBuffer_[i].y), 0.6f));
		color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, dataBuffer_[i].z), 0.6f));
		fwrite(color, 1, 3, fp);
	}
	fclose(fp);

	// Python脚本的路径，如果不在PATH中，需要提供完整路径  
	const char* pythonScriptPath = "python/ppm2jpg.py";
	// PPM图片的路径  
	const char* ppmImagePath = "./output/binary.ppm";
	// 输出的JPG图片路径  
	const char* jpgOutputPath = "./output/output.jpg";

	std::string command = "python " + std::string(pythonScriptPath) + " " + std::string(ppmImagePath) + " " +
		std::string(jpgOutputPath);
	// 执行命令行指令  
	int result = system(command.c_str());

	// 检查执行结果  
	if (result == 0) {
		std::cout << "Python script executed successfully." << std::endl;
	}
	else {
		std::cerr << "Failed to execute Python script." << std::endl;
	}
}

void Scene::useMsaa(int sizeH, int sizeV)
{
	useMsaa_ = true;
	msaaH_ = sizeH;
	msaaV_ = sizeV;
	msaaSpp_ = sizeH * sizeV;
	//zBuffer_.clear();
	zBufferMsaa_.resize(width_ * height_);
	for (auto& item : zBufferMsaa_)
	{
		item.resize(msaaSpp_);
		std::fill(item.begin(), item.end(), std::numeric_limits<float>::infinity());
	}
	
}




