#pragma once
#include <cmath>
#include <iostream>
#include <algorithm>
#include <vector>
#include <Eigen/Eigen>
#include <opencv2/core.hpp>

/*
向量的一些常用计算，包括向量长度，归一化，点成，叉乘，以及一些常见的运算符重载
*/


template<typename T>
class Vec2
{
public:
	T x, y;
public:
	Vec2() :x(0), y(0) {}
	Vec2(T xx) :x(xx), y(xx) {}
	Vec2(T xx, T yy) :x(xx), y(yy) {}
	Vec2 operator* (const T& r)const { return Vec2(x * r, y * r); }
	Vec2 operator/ (const T& r)const { return Vec2(x / r, y / r); }
	Vec2 operator+ (const Vec2& v)const { return Vec2(x + v.x, y + v.y); }
	T norm() { return std::sqrt(x * x + y * y); }
	T norm2() { return (x * x + y * y); }
	Vec2& normalize() { *this = (*this) / norm(); return *this; }


	friend Vec2 operator*(const T& val, const Vec2& v)
	{
		return v.operator*(val);
	}

	bool strictSmall(const Vec2& in) const
	{
		return x <= in.x && y <= in.y;
	}

	bool strictBig(const Vec2& in) const
	{
		return x >= in.x && y >= in.y;
	}
};

template<typename T>
class Vec3
{
public:
	Vec3() : x(0), y(0), z(0) {}
	Vec3(T xx) :x(xx), y(xx), z(xx) {};
	Vec3(T xx, T yy, T zz) :x(xx), y(yy), z(zz) {}
	Vec3(Vec2<T> xy,T z):x(xy.x),y(xy.y),z(z){}
	Vec3(T x, Vec2<T> yz):x(x),y(yz.y),z(yz.z){}
	Vec3 operator* (const T& r)const { return Vec3(x * r, y * r, z * r); }
	Vec3 operator/ (const T& r)const { return Vec3(x / r, y / r, z / r); }
	Vec3 operator+ (const Vec3& v)const { return Vec3(x + v.x, y + v.y, z + v.z); }
	Vec3 operator- (const Vec3& v)const { return Vec3(x - v.x, y - v.y, z - v.z); }
	Vec3 operator- ()const { return Vec3(-x, -y, -z); }
	Vec3& operator+= (const Vec3& v) {
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}

	Vec3& operator-= (const Vec3& v) {
		return this->operator+=(-v);
	}

	bool operator==(const Vec3& other) const
	{
		return (this->x == other.x && this->y == other.y && this->z == other.z);
	}

	// Bool Not Equals Operator Overload
	bool operator!=(const Vec3& other) const
	{
		return !operator==(other);
	}

	Vec3 operator/= (const T& r) {
		return operator/(r);
	}

	Vec3 operator*= (const T& r) {
		return operator*(r);
	}

	friend std::ostream& operator << (std::ostream& os, const Vec3& v)
	{
		return os << v.x << ", " << v.y << ", " << v.z;
	}

	T operator[](int index) const {
		return (&x)[index];
	}

	T norm() const { return std::sqrt(x * x + y * y + z * z); }
	T norm2() const  { return (x * x + y * y + z * z); }
	Vec3& normalize() { (*this) = (*this) / norm(); return (*this); }
	Vec3 lerp (const Vec3& a, const Vec3& b, const T& t) const
	{
		return a * (1 - t) + b * t;
	}

	T dot(const Vec3& b) const
	{
		return dotProduct(*this, b);
	}

	static T dotProduct(const Vec3& a, const Vec3& b)
	{
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	Vec3 cross(const Vec3& b) const
	{
		return crossProduct(*this, b);
	}

	static Vec3 crossProduct(const Vec3& a, const Vec3& b)
	{
		return Vec3(
			a.y * b.z - a.z * b.y,
			a.z * b.x - a.x * b.z,
			a.x * b.y - a.y * b.x
		);
	}

	// Projection Calculation of a onto b
	Vec3 project2Vec(Vec3 b) const
	{
		Vec3 bn = b.normalize();
		return bn * (*this).dot(bn);
	}

	T angleBetweenVec(const Vec3& b)
	{
		T angle = (*this).dot(b);
		angle /= this->norm() * b.norm();
		return angle = acosf(angle);
	}

	friend Vec3 operator*(const T& val, const Vec3& v)
	{
		return v.operator*(val);
	}

	bool strictSmall(const Vec3& in) const
	{
		return x <= in.x && y <= in.y && z <= in.z;
	}

	bool strictBig(const Vec3& in) const
	{
		return x >= in.x && y >= in.y && z >= in.z;
	}

	Eigen::Vector4f to_vec4(const T val)const 
	{
		return std::move(Eigen::Vector4f{ x, y, z, val });
	}

	Eigen::Vector3f to_vec3()
	{
		return Eigen::Vector3f(x, y, z);
	}
	cv::Vec<T,3> toCV3()
	{
		return cv::Vec<T, 3>{x, y, z};
	}

	void reset()
	{
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	}

	T X() { return this->x; }
	T Y() { return this->y; }
	T Z() { return this->z; }


public:
	T x, y, z;
};


using Vec2f = Vec2<float>;
using Vec2i = Vec2<int>;

using Vec3f = Vec3<float>;
using Vec3i = Vec3<int>;


