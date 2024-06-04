#pragma once
#include <iostream>
#include <cmath>
#include <random>
#include <tuple>
#include "Vector.hpp"

#undef M_PI
#define M_PI 3.141592653589793f


float clamp(const float& lo, const float& hi, const float& v);
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vec3f* v);
static bool insideTriangle(const Vec2f& pos, const Vec3f* v);
static bool insideTriangle(float x, float y, const Vec3f* v);
void UpdateProgress(float progress);



namespace Color
{
    const Vec3f Red = Vec3f(1.0f, 0.0f, 0.0f);
    const Vec3f Green = Vec3f(0.0f, 1.0f, 0.0f);
    const Vec3f Blue = Vec3f(0.0f, 0.0f, 1.0f);
    const Vec3f White = Vec3f(1.0f, 1.0f, 1.0f);
    const Vec3f Black = Vec3f(0.0f, 0.0f, 0.0f);
};


inline float clamp(const float& lo, const float& hi, const float& v)
{
	return std::max(lo, std::min(hi, v));
}
template<typename T>
inline  T mixture(const T& l, const T& r,float t)
{
    return l * t + (1 - t) * r;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vec3f* v)
{
    float c1 = (x * (v[1].y - v[2].y) + (v[2].x - v[1].x) * y + v[1].x * v[2].y - v[2].x * v[1].y) / (v[0].x * (v[1].y - v[2].y) + (v[2].x - v[1].x) * v[0].y + v[1].x * v[2].y - v[2].x * v[1].y);
    float c2 = (x * (v[2].y - v[0].y) + (v[0].x - v[2].x) * y + v[2].x * v[0].y - v[0].x * v[2].y) / (v[1].x * (v[2].y - v[0].y) + (v[0].x - v[2].x) * v[1].y + v[2].x * v[0].y - v[0].x * v[2].y);
    float c3 = (x * (v[0].y - v[1].y) + (v[1].x - v[0].x) * y + v[0].x * v[1].y - v[1].x * v[0].y) / (v[2].x * (v[0].y - v[1].y) + (v[1].x - v[0].x) * v[2].y + v[0].x * v[1].y - v[1].x * v[0].y);
    return { c1,c2,c3 };
}

static bool insideTriangle(const Vec2f& pos, const Vec3f* v)
{
    return insideTriangle(pos.x, pos.y, v);
}

static bool insideTriangle(float x,float y,const Vec3f* v)
{
	const Vec3f line1(v[0].x - x, v[0].y - y, 0);
	const Vec3f line2(v[1].x - x, v[1].y - y, 0);
	const Vec3f line3(v[2].x - x, v[2].y - y, 0);

	const float z1 = line1.cross(line2).z;
	const float z2 = line2.cross(line3).z;
	const float z3 = line3.cross(line1).z;
    if ((z1 <= 0 && z2 <= 0 && z3 <= 0) || (z1 >= 0 && z2 >= 0 && z3 >= 0))
    {
        return true;
    }
    return false;
}

inline void UpdateProgress(float progress)
{
	const int barWidth = 70;

    std::cout << "[";
	const int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
}


inline std::string tail(const std::string& in)
{
	const size_t token_start = in.find_first_not_of(" \t");
	const size_t space_start = in.find_first_of(" \t", token_start);
	const size_t tail_start = in.find_first_not_of(" \t", space_start);
	const size_t tail_end = in.find_last_not_of(" \t");
    if (tail_start != std::string::npos && tail_end != std::string::npos)
    {
        return in.substr(tail_start, tail_end - tail_start + 1);
    }
    else if (tail_start != std::string::npos)
    {
        return in.substr(tail_start);
    }
    return "";
}

inline std::string firstToken(const std::string& in)
{
    if (!in.empty())
    {
	    const size_t token_start = in.find_first_not_of(" \t");
	    const size_t token_end = in.find_first_of(" \t", token_start);
        if (token_start != std::string::npos && token_end != std::string::npos)
        {
            return in.substr(token_start, token_end - token_start);
        }
        else if (token_start != std::string::npos)
        {
            return in.substr(token_start);
        }
    }
    return "";
}

inline void split(const std::string& in,
    std::vector<std::string>& out,
    std::string token)
{
    out.clear();

    std::string temp;

    for (int i = 0; i < int(in.size()); i++)
    {
        std::string test = in.substr(i, token.size());

        if (test == token)
        {
            if (!temp.empty())
            {
                out.push_back(temp);
                temp.clear();
                i += (int)token.size() - 1;
            }
            else
            {
                out.push_back("");
            }
        }
        else if (i + token.size() >= in.size())
        {
            temp += in.substr(i, token.size());
            out.push_back(temp);
            break;
        }
        else
        {
            temp += in[i];
        }
    }
}

// Get element at given index position
template <class T>
inline const T& getElement(const std::vector<T>& elements, std::string& index)
{
    int idx = std::stoi(index);
    if (idx < 0)
        idx = int(elements.size()) + idx;
    else
        idx--;
    return elements[idx];
}

inline bool SameSide(Vec3f p1, Vec3f p2, Vec3f a, Vec3f b)
{
	    const Vec3f cp1 = (b - a).cross(p1 - a);
	    const Vec3f cp2 = (b - a).cross(p2 - a);

        if (cp1.dot(cp2) >= 0)
            return true;
        else
            return false;
}

    // Generate a cross produect normal for a triangle
inline Vec3f GenTriNormal(Vec3f t1, Vec3f t2, Vec3f t3)
{
        Vec3f u = t2 - t1;
        Vec3f v = t3 - t1;

        const Vec3f& normal = Vec3<float>::crossProduct(u, v);

        return normal;
}

    // Check to see if a Vec3f Point is within a 3 Vec3f Triangle
inline  bool inTriangle(Vec3f point, Vec3f tri1, Vec3f tri2, Vec3f tri3)
{
        // Test to see if it is within an infinite prism that the triangle outlines.
        bool within_tri_prisim = SameSide(point, tri1, tri2, tri3) && SameSide(point, tri2, tri1, tri3)
            && SameSide(point, tri3, tri1, tri2);

        // If it isn't it will never be on the triangle
        if (!within_tri_prisim)
            return false;

        // Calulate Triangle's Normal
        Vec3f n = GenTriNormal(tri1, tri2, tri3);

        // Project the point onto this normal
        Vec3f proj = point.project2Vec(n);

        // If the distance from the triangle to the point is 0
        //	it lies on the triangle
        if (proj.norm2() == 0)
            return true;
        else
            return false;
}