#pragma once
#include <Vector.hpp>

class Ray
{
public:
    Vec3<float> ori;
    Vec3<float> dir;

    float t;
    float t_min, t_max;

    Ray(const Vec3f& origin, const Vec3f& direction, const float _t = 0.0) : ori(origin), dir(direction), t(_t) {
        t_min = 0.0;
        t_max = std::numeric_limits<float>::max();
    }

    Vec3f operator()(float t) const { return ori + dir * t; }

    friend std::ostream& operator<<(std::ostream& os, const  Ray& r) {
        os << "[origin:=" << r.ori << ", direction=" << r.dir << ", time=" << r.t << "]\n";
        return os;
    }
};
