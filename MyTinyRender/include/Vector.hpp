#pragma once
#include <cmath>
#include <iostream>
#include <algorithm>
#include <vector>

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
	friend std::ostream& operator << (std::ostream& os, const Vec3& v)
	{
		return os << v.x << ", " << v.y << ", " << v.z;
	}

	double operator[](int index) const {
		return (&x)[index];
	}

	T norm() { return std::sqrt(x * x + y * y + z * z); }
	T norm2() { return (x * x + y * y + z * z); }
	Vec3& normalize() { *this = (*this) / norm(); return *this; }
	Vec3 lerp(const Vec3& a, const Vec3& b, const T& t)
	{
		return a * (1 - t) + b * t;
	}

	T dotProduct(const Vec3& a, const Vec3& b)
	{
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	Vec3 crossProduct(const Vec3& a, const Vec3& b)
	{
		return Vec3(
			a.y * b.z - a.z * b.y,
			a.z * b.x - a.x * b.z,
			a.x * b.y - a.y * b.x
		);
	}

	friend Vec3 operator*(const T& val, const Vec3& v)
	{
		return v.operator*(val);
	}
public:
	T x, y, z;
};


using Vec2f = Vec2<float>;
using Vec2i = Vec2<int>;

using Vec3f = Vec3<float>;
using Vec3i = Vec3<int>;


