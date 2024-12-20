#ifndef GLOBAL_HPP
#define GLOBAL_HPP


#include <iostream>
#include <cmath>
#include <random>
#include <tuple>
#include "Vector.hpp"

#undef M_PI
#define M_PI 3.141592653589793f
#define EPSILON 1e-5


namespace Color
{
    const Vec3f Red = Vec3f(1.0f, 0.0f, 0.0f);
    const Vec3f Green = Vec3f(0.0f, 1.0f, 0.0f);
    const Vec3f Blue = Vec3f(0.0f, 0.0f, 1.0f);
    const Vec3f White = Vec3f(1.0f, 1.0f, 1.0f);
    const Vec3f Black = Vec3f(0.0f, 0.0f, 0.0f);
    const Vec3f Gray = Vec3f(0.5f, 0.5f, 0.5f);
};

class GamesMath
{
public:
    static inline float clamp(const float& lo, const float& hi, const float& v)
    {
        return std::max(lo, std::min(hi, v));
    }

    static Vec3f reflect(const Vec3f& I, const Vec3f& N)
    {
        return I - 2 * I.dot(N) * N;
    }

    static Vec3f refract(const Vec3f& I, const Vec3f& N, const float& ior)
    {
        float cosi = clamp(-1, 1, I.dot(N));
        float etai = 1, etat = ior;
        Vec3f n = N;
        if (cosi < 0) { cosi = -cosi; }
        else { std::swap(etai, etat); n = -N; }
        float eta = etai / etat;
        float k = 1 - eta * eta * (1 - cosi * cosi);
        return k < 0 ? 0 : eta * I + (eta * cosi - sqrtf(k)) * n;
    }



    static float fresnel(const Vec3f& I, const Vec3f& N, const float& ior)
    {
        float kr;
        float cosi = clamp(-1, 1, I.dot(N));
        float etai = 1, etat = ior;
        if (cosi > 0) { std::swap(etai, etat); }
        // Compute sini using Snell's law
        float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
        // Total internal reflection
        if (sint >= 1) {
            kr = 1;
        }
        else {
            float cost = sqrtf(std::max(0.f, 1 - sint * sint));
            cosi = fabsf(cosi);
            float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
            float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
            kr = (Rs * Rs + Rp * Rp) / 2;
        }
        return kr;
    }

    //将局部方向转换为世界方向
    static Vec3f toWorld(const Vec3f& a, const Vec3f& N) {
        Vec3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)) {
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vec3f(N.z * invLen, 0.0f, -N.x * invLen);
        }
        else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vec3f(0.0f, N.z * invLen, -N.y * invLen);
        }
        B = Vec3f::crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

    template<typename T>
    static inline  T mixture(const T& l, const T& r, float t)
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

    static bool insideTriangle(float x, float y, const Vec3f* v)
    {
        //return inTriangle(Vec3f(x, y, 0), v[0], v[1], v[2]);
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

    static inline void UpdateProgress(float progress)
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


    static inline std::string tail(const std::string& in)
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

    static inline std::string firstToken(const std::string& in)
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

    static inline void split(const std::string& in,
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
    static inline const T& getElement(const std::vector<T>& elements, std::string& index)
    {
        int idx = std::stoi(index);
        if (idx < 0)
            idx = int(elements.size()) + idx;
        else
            idx--;
        return elements[idx];
    }

    static inline bool SameSide(Vec3f p1, Vec3f p2, Vec3f a, Vec3f b)
    {
        const Vec3f cp1 = (b - a).cross(p1 - a);
        const Vec3f cp2 = (b - a).cross(p2 - a);

        if (cp1.dot(cp2) >= 0)
            return true;
        else
            return false;
    }

    // Generate a cross produect normal for a triangle
    static inline Vec3f GenTriNormal(Vec3f t1, Vec3f t2, Vec3f t3)
    {
        Vec3f u = t2 - t1;
        Vec3f v = t3 - t1;

        const Vec3f& normal = Vec3<float>::crossProduct(u, v);

        return normal;
    }

    // Check to see if a Vec3f Point is within a 3 Vec3f Triangle
    static inline  bool inTriangle(Vec3f point, Vec3f tri1, Vec3f tri2, Vec3f tri3)
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


    static inline  bool solveQuadratic(const float& a, const float& b, const float& c, float& x0, float& x1)
    {
        float discr = b * b - 4 * a * c;
        if (discr < 0) return false;
        else if (discr == 0) x0 = x1 = -0.5 * b / a;
        else {
            float q = (b > 0) ?
                -0.5 * (b + sqrt(discr)) :
                -0.5 * (b - sqrt(discr));
            x0 = q / a;
            x1 = c / q;
        }
        if (x0 > x1) std::swap(x0, x1);
        return true;
    }

    static inline float deg2rad(float deg)
    {
        return deg / 180.0 * M_PI;
    }

    static inline float get_random_float()
    {
        std::random_device dev;
        std::mt19937 rng(dev());
        std::uniform_real_distribution<float> dist(0.f, 1.f); // distribution in range [1, 6]

        return dist(rng);
    }

    static inline float P_RR()
    {
        float q = get_random_float();
        float p = get_random_float();
        if (p > q)
        {
            return p;
        }
        return 0;
    }
};



#endif // !GLOBAL_HPP
