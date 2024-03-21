#pragma once
#include <iostream>
#include <cmath>
#include <random>
#include <tuple>
#include "Vector.hpp"

#undef M_PI
#define M_PI 3.141592653589793f

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

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vec3f* v)
{
    float c1 = (x * (v[1].y - v[2].y) + (v[2].x - v[1].x) * y + v[1].x * v[2].y - v[2].x * v[1].y) / (v[0].x * (v[1].y - v[2].y) + (v[2].x - v[1].x) * v[0].y + v[1].x * v[2].y - v[2].x * v[1].y);
    float c2 = (x * (v[2].y - v[0].y) + (v[0].x - v[2].x) * y + v[2].x * v[0].y - v[0].x * v[2].y) / (v[1].x * (v[2].y - v[0].y) + (v[0].x - v[2].x) * v[1].y + v[2].x * v[0].y - v[0].x * v[2].y);
    float c3 = (x * (v[0].y - v[1].y) + (v[1].x - v[0].x) * y + v[0].x * v[1].y - v[1].x * v[0].y) / (v[2].x * (v[0].y - v[1].y) + (v[1].x - v[0].x) * v[2].y + v[0].x * v[1].y - v[1].x * v[0].y);
    return { c1,c2,c3 };
}


inline void UpdateProgress(float progress)
{
    int barWidth = 70;

    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
};