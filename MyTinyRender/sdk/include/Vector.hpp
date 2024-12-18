#pragma once
#include <cmath>
#include <iostream>
#include <algorithm>
#include <vector>

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
	Vec3(Vec2<T> xy, T z) :x(xy.x), y(xy.y), z(z) {}
	Vec3(T x, Vec2<T> yz) :x(x), y(yz.y), z(yz.z) {}
	Vec3 operator* (const T& r)const { return Vec3(x * r, y * r, z * r); }
	Vec3 operator* (const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); }
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

	Vec3& operator/= (const T& r) {
		x /= r;
		y /= r;
		z /= r;
		return *this;
	}

	Vec3& operator*= (const T& r) {
		x *= r;
		y *= r;
		z *= r;
		return *this;
	}

	friend std::ostream& operator << (std::ostream& os, const Vec3& v)
	{
		return os << v.x << ", " << v.y << ", " << v.z;
	}

	T operator[](int index) const {
		return (&x)[index];
	}

	T norm() const { return std::sqrt(x * x + y * y + z * z); }
	T norm2() const { return (x * x + y * y + z * z); }
	Vec3& normalize() { (*this) = (*this) / norm(); return (*this); }

	static Vec3 lerp(const Vec3& a, const Vec3& b, const T& t)
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

	cv::Vec<T, 3> toCV3()
	{
		return cv::Vec<T, 3>{x, y, z};
	}

	void reset()
	{
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	}

	static Vec3 vecMultiplication(const Vec3& vec1, const Vec3& vec2)
	{
		return Vec3(vec1.x * vec2.x, vec1.y * vec2.y, vec1.z * vec2.z);
	}

	T& X() { return this->x; }
	T& Y() { return this->y; }
	T& Z() { return this->z; }


public:
	T x, y, z;
};


template<typename T>
class Vec4
{
public:
	T x, y, z, w;

public:
	Vec4() : x(0), y(0), z(0), w(0) {}
	Vec4(T xx) :x(xx), y(xx), z(xx), w(xx) {}
	Vec4(T xx, T yy, T zz, T ww) :x(xx), y(yy), z(zz), w(ww) {}
	Vec4(T x, Vec3<T> xyz) :x(x), y(xyz.x), z(xyz.y), w(xyz.z) {}
	Vec4(Vec3<T> xyz, T w) :x(xyz.x), y(xyz.y), z(xyz.z), w(w) {}
	Vec4 operator* (const T& r)const { return Vec4(x * r, y * r, z * r, w * r); }
	Vec4 operator/ (const T& r)const { return Vec4(x / r, y / r, z / r, w / r); }
	Vec4 operator+ (const Vec4& v)const { return Vec4(x + v.x, y + v.y, z + v.z, w + v.w); }
	Vec4 operator- (const Vec4& v)const { return Vec4(x - v.x, y - v.y, z - v.z, w - v.w); }
	Vec4 operator- ()const { return Vec4(-x, -y, -z, -w); }
	Vec4& operator+= (const Vec4& v) {
		x += v.x;
		y += v.y;
		z += v.z;
		w += v.w;
		return *this;
	}


	Vec4& operator-= (const Vec4& v) {
		return this->operator+=(-v);
	}

	bool operator==(const Vec4& other) const
	{
		return (this->x == other.x && this->y == other.y && this->z == other.z && this->w == other.w);
	}

	// Bool Not Equals Operator Overload
	bool operator!=(const Vec4& other) const
	{
		return !operator==(other);
	}

	Vec4& operator/= (const T& r) {
		x /= r;
		y /= r;
		z /= r;
		w /= r;
		return *this;
	}

	Vec4& operator*= (const T& r) {
		x *= r;
		y *= r;
		z *= r;
		w *= r;
		return *this;
	}

	friend std::ostream& operator << (std::ostream& os, const Vec4& v)
	{
		return os << v.x << ", " << v.y << ", " << v.z << ", " << v.w;
	}

	T& operator[](int index)
	{
		return (&x)[index];
	}

	T operator[](int index) const {
		return (&x)[index];
	}

	Vec3<T> head() const
	{
		return Vec3<T>(x, y, z);
	}

	T& X() { return this->x; }
	T& Y() { return this->y; }
	T& Z() { return this->z; }
	T& W() { return this->w; }

};

template<typename T>
class Matrix4x4 
{
public:
	std::vector<std::vector<T>> mat;

public:
	// 构造函数，初始化为4x4的矩阵，所有元素为0  
	Matrix4x4() : mat(4, std::vector<T>(4, 0)) {}
	
	Matrix4x4(const Matrix4x4& m):Matrix4x4()
	{
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				mat[i][j] = m(i,j);
			}
		}
	}

	Matrix4x4(
		T a00, T a01, T a02, T a03,
		T a10, T a11, T a12, T a13,
		T a20, T a21, T a22, T a23,
		T a30, T a31, T a32, T a33):Matrix4x4()
	{
		mat[0][0] = a00;
		mat[0][1] = a01;
		mat[0][2] = a02;
		mat[0][3] = a03;

		mat[1][0] = a10;
		mat[1][1] = a11;
		mat[1][2] = a12;
		mat[1][3] = a13;

		mat[2][0] = a20;
		mat[2][1] = a21;
		mat[2][2] = a22;
		mat[2][3] = a23;

		mat[3][0] = a30;
		mat[3][1] = a31;
		mat[3][2] = a32;
		mat[3][3] = a33;
	}

	// 索引运算符重载，用于访问矩阵元素  
	T& operator()(int row, int col) 
	{
		if (row < 0 || row >= 4 || col < 0 || col >= 4) {
			throw std::out_of_range("Index out of bounds for 4x4 matrix");
		}
		return mat[row][col];
	}

		

	// 常量索引运算符重载，用于访问矩阵元素（const版本）  
	const T& operator()(int row, int col) const 
	{
		if (row < 0 || row >= 4 || col < 0 || col >= 4) {
			throw std::out_of_range("Index out of bounds for 4x4 matrix");
		}
		return mat[row][col];
	}

	T* operator[](const int i)
	{
		return mat[i].data();
	}

	

	Matrix4x4 operator-() const
	{
		Matrix4x4 result;
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				result(i, j) = -mat(i, j);
			}
		}
		return result;
	}

	// 矩阵加法运算符重载  
	Matrix4x4 operator+(const Matrix4x4& other) const 
	{
		Matrix4x4 result;
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				result(i, j) = this->operator()(i, j) + other(i, j);
			}
		}
		return result;
	}

	Matrix4x4& operator+=(const Matrix4x4& other)
	{
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				mat[i][j] += other(i, j);
			}
		}
		return *this;
	}

	Matrix4x4& operator-= (const Matrix4x4& m) {
		return this->operator+=(-m);
	}

	Matrix4x4 operator-(const Matrix4x4& other) const
	{
		return operator+(-other);
	}

	Matrix4x4 operator*(const Matrix4x4& m) 
	{
		Matrix4x4 ansM;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				for (int k = 0; k < 4; k++) {
					ansM.mat[i][j] += mat[i][k] * m.mat[k][j];
				}
			}
		}
		return ansM;
	}

	Vec4<T> operator*(const Vec4<T>& v) const
	{
		Vec4<T> retV;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				retV[i] += mat[i][j] * v[j];
			}
		}
		return retV;
	}

	/*Vec4<T> operator*(const Vec4<T>& v) const
	{
		Vec4<T> retV;
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				retV[i] += mat[i][j] * v[j];
			}
		}
		return retV;
	}*/

	Matrix4x4& identity()
	{
		for (int i = 0; i < 4; ++i) 
		{
			for (int j = 0; j < 4; ++j)
			{
				if (i == j)
				{
					mat[i][j] = static_cast<T>(1);
				}
				else
					mat[i][j] = 0;
			}
		}
		return *this;
	}

	static Matrix4x4 Identity()
	{
		Matrix4x4 ansM;
		return ansM.identity();
	}

	Matrix4x4& operator*=(const Matrix4x4& m)
	{
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				for (int k = 0; k < 4; k++) {
					mat[i][j] += mat[i][k] * m.mat[k][j];
				}
			}
		}
		return *this;
	}

	// 矩阵转置  
	Matrix4x4 transpose() const 
	{
		Matrix4x4 ansM;
		for (size_t i = 0; i < 4; ++i) {
			for (size_t j = 0; j < 4; ++j) {
				ansM[j][i] = mat[i][j];
			}
		}
		return ansM;
	}

	//交换矩阵的两行
	void SwapRow(int a, int b)
	{
		for (int i = 0; i < 4; i++) {
			std::swap(mat[a][i], mat[b][i]);
		}
	}

	void LineMultiK(int a, double k) 
	{
		for (int i = 0; i < 4; i++)
			mat[a][i] *= k;
	}

	Vec4<T> getRow(int i)
	{
		return Vec4<T>(mat[i][0], mat[i][1], mat[i][2], mat[i][3]);
	}

	Vec4<T> getCol(int i)
	{
		return Vec4<T>(mat[0][i], mat[1][i], mat[2][i], mat[3][i]);
	}

	//矩阵求逆
	Matrix4x4 Inverse() {
		Matrix4x4 tmpM(mat);
		Matrix4x4 E;
		E.identity();
		//首先将tmpM转换为上三角矩阵
		for (int i = 0; i < 4; i++) {
			//判断对角方向的元素是否为0
			int j = i;
			while (mat[i][j] == 0 && j < 4)j++;
			if (j == 4) {
				std::cout << "this matrix has no inverse!" << std::endl;
				return *this;
			}
			else if (i != j)
			{
				tmpM.SwapRow(i, j);
				E.SwapRow(i, j);
			}
				
			//将对角位置转换为1
			E.LineMultiK(i, 1.0 / tmpM(i, i));
			tmpM.LineMultiK(i, 1.0 / tmpM(i, i));
			//将该列非对角位置转换为0
			for (int k = 0; k < 4; k++) {
				if (k == i)
					continue;
				for (int t = i + 1; t < 4; t++)
				{
					tmpM[k][t] -= tmpM(i, t) * tmpM(k, i);
				}
				for (int t = 0; t < 4; t++)
				{
					E[k][t] -= E(i, t) * tmpM(k, i);
				}
				tmpM[k][i] = 0;
			}
		}
		return E;
	}


	

	// 矩阵输出（使用友元函数重载<<运算符）  
	friend std::ostream& operator<<(std::ostream& os, const Matrix4x4& m)
	{
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				os << m(i, j) << " ";
			}
			os << std::endl;
		}
		return os;
	}
};





using Vec2f = Vec2<float>;
using Vec2i = Vec2<int>;

using Vec3f = Vec3<float>;
using Vec3d = Vec3<double>;
using Vec3i = Vec3<int>;

using Vec4f = Vec4<float>;
using Vec4i = Vec4<int>;

using Vec3Vector = std::vector<Vec3f>;
using Vec4Vector = std::vector<Vec4f>;


using Matrix4x4f = Matrix4x4<float>;
