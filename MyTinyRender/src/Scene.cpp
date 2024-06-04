#include "Scene.h"
#include "global.hpp"
#include "Geometry.h"
#include "Rasterizer.h"
#include "Texture.h"
#include "Mesh.h"

#include <Eigen/Eigen>

#include "Camera.h"

struct fragment_shader_payload
{
	fragment_shader_payload()
	{
		texture = nullptr;
	}

	fragment_shader_payload(const Eigen::Vector3f& col, const Eigen::Vector3f& nor, const Eigen::Vector2f& tc, Texture* tex) :
		color(col), normal(nor), tex_coords(tc), texture(tex) {}


	Eigen::Vector3f view_pos;
	Eigen::Vector3f color;
	Eigen::Vector3f normal;
	Eigen::Vector2f tex_coords;
	Texture* texture;
};



Scene::Scene()
{
	dataBuffer_.resize(width_ * height_);
	zBuffer_.resize(width_ * height_);
	std::fill(zBuffer_.begin(), zBuffer_.end(), std::numeric_limits<float>::infinity());

	rasterizer_ = std::make_shared<Rasterizer>(this);
	camera_ = std::make_shared<Camera>(this);
}


void Scene::addMesh(const Mesh& mesh)
{
	std::vector<Geometry*>geometries;
	for (int i = 0; i < mesh.vertices_.size(); i += 3)
	{
		Triangle* t = new Triangle();
		for (int j = 0; j < 3; j++)
		{
			t->setVertex(j, Vec3f(mesh.vertices_[i + j].position_.x, mesh.vertices_[i + j].position_.y, mesh.vertices_[i + j].position_.z));
			t->setNormal(j, Vec3f(mesh.vertices_[i + j].normal_.x, mesh.vertices_[i + j].normal_.y, mesh.vertices_[i + j].normal_.z));
			t->setTexCoord(j, Vec2f(mesh.vertices_[i + j].textureCoordinate_.x, mesh.vertices_[i + j].textureCoordinate_.y));
		}
		t->boxConstruct();
		geometries.push_back(t);
	}
	auto pair = std::make_pair(&mesh, geometries);
	meshCords_.insert(pair);
}

void Scene::doUpDate()
{
	for(const auto& it:meshCords_)
	{
		for (const auto& model : it.second)
		{
			const Mesh* mesh = it.first;
			auto* triangle = dynamic_cast<Triangle*>(model);
			if (!triangle)
				return;
			////投影变换

			////视口变化
			/////绕(0,1,1)旋转45°
			//Eigen::Vector3f t0(0,1.0,1);// 初始向量
			//Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

			//// 在 X 轴上定义一个 0.5 米的平移.
			//transform_2.translation() << 0.5, -1.0, 0.0;

			//// 和前面一样的旋转; Z 轴上旋转 theta 弧度
			//transform_2.rotate(Eigen::AngleAxisf(M_PI / 6, t0));



			//Eigen::Matrix4f modelMatrix = transform_2.matrix();
			//
			//Eigen::Matrix4f viewMatrix = setViewMatrix(Vec3f(1, 0.5f,12.f));
			//Eigen::Matrix4f pojMatrix = projectMatrix();


			//Eigen::Matrix4f mvp = pojMatrix * viewMatrix * modelMatrix;

			//std::vector<Eigen::Vector4f> vec = triangle->setModelMatrix(mvp);
			//viewportMatrix(vec);

			std::vector<Eigen::Vector4f> vec = getViewPortPos(triangle->getVertexs(), mesh);
			//光栅化
			Triangle t;
			for (int i = 0; i < 3; ++i)
			{
				Vec3f v = { vec[i].x(),vec[i].y(),vec[i].z() };

				t.setVertex(i, v);
				t.setColor(i, triangle->getColor(i));
				t.setTexCoord(i, triangle->getTexCoord(i));
			}
			t.boxConstruct();
			rasterizeTriangle(&t, mesh);
		}
	}
}

void Scene::rasterizeTriangle(Geometry* geometry,const Mesh* mesh)
{
	Triangle* triangle = dynamic_cast<Triangle*>(geometry);
	if(!triangle)
		return;

	for(int i=0;i< width_;i++)
	{
		for(int j=0;j<height_;j++)
		{
			rasterizer_->rasterizeTriangle(i, j, triangle, mesh);
		}
	}
}


std::vector<Eigen::Vector4f> Scene::getViewPortPos(const std::vector<Vec3f>& trianglePos, const Mesh* mesh) const
{
	Eigen::Matrix4f mvp = camera_->getProjectionMat() * camera_->getViewMat() * mesh->modelMatrix_;

	std::vector<Eigen::Vector4f>res;
	for (auto& p : trianglePos)
	{
		Eigen::Vector4f vec4f = mvp * p.to_vec4(1.0);
		res.push_back(vec4f);
	}
	viewportMatrix(res);
	return std::move(res);
}

void  Scene::viewportMatrix(std::vector<Eigen::Vector4f>& vec) const
{
	float f1 = (far_ + near_) / 2.0f;
	float f2 = (far_ - near_) / 2.0f;

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

void Scene::clear()
{
	std::fill(dataBuffer_.begin(), dataBuffer_.end(), Vec3f{ 0, 0, 0 });
	std::fill(zBuffer_.begin(), zBuffer_.end(), std::numeric_limits<float>::infinity());
	if(useMsaa_)
	{
		for (auto& item : zBufferMsaa_)
		{
			item.resize(msaaSpp_);
			std::fill(item.begin(), item.end(), std::numeric_limits<float>::infinity());
		}
	}
	
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




