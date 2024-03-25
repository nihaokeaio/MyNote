#include "Geometry.h"
#include "Scene.h"

void Line::setColor(Vec3f c)
{
    color = c;
}

void Line::line(Vec2i p0, Vec2i p1, Scene* scene, Vec3f color)
{
	line(p0.x, p0.y, p1.x, p1.y, scene, color);
}

void Line::line(int x0, int y0, int x1, int y1, Scene* scene, Vec3f color)
{
    // =是否是陡峭的，即斜率是否大于1,置换后斜率必然小于1
    bool steep = false;
    if (abs(x0 - x1) < abs(y0 - y1))
    {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }

    //考虑不同方向的线段，都从x较小的点开始绘制
    if (x0 > x1)
    {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    int dx = (x1 - x0);
    int dy = (y1 - y0);
    int derror2 = abs(dy * 2);
    int error = 0;
    int y = y0;
    for (int x = x0; x <= x1; x++)
    {
        if (steep)
        {
            scene->set_data_buffer(y, x, color);
        }
        else
        {
            scene->set_data_buffer(x, y, color);
        }
        error += derror2;
        //前面已经保证了斜率小于1
        if (error > dx)
        {
            y += (y1 > y0 ? 1 : -1);
            error -= 2 * dx;
        }
    }
}

void Line::draw(Scene* scene) const
{
    Vec2i p0 = { (int)point[0].x,(int)point[0].y };
    Vec2i p1 = { (int)point[1].x,(int)point[1].y };
    line(p0, p1, scene, color);
}

void Triangle::setColor(Vec3f c)
{
	const Vec3f c3[3] = {c,c,c};
    setColors(c3);
}

void Triangle::setColors(const Vec3f* c)
{
    color_[0] = c[0];
    color_[1] = c[1];
    color_[2] = c[2];
}

void Triangle::draw(Scene* scene) const
{
    Vec2i p0 = { (int)point_[0].x,(int)point_[0].y };
    Vec2i p1 = { (int)point_[1].x,(int)point_[1].y };
    Vec2i p2 = { (int)point_[2].x,(int)point_[2].y };

    Line::line(p0, p1, scene, color_[0]);
    Line::line(p1, p2, scene, color_[1]);
    Line::line(p2, p0, scene, color_[2]);
}

std::vector<Eigen::Vector4f> Triangle::setModelMatrix(Eigen::Matrix4f mat)
{
    std::vector<Eigen::Vector4f>res;
    for(auto& p:point_)
    {
        Eigen::Vector4f vec4f = mat * p.to_vec4(1.0);
        p.x = vec4f.x();
        p.y = vec4f.y();
        p.z = vec4f.z();
        res.push_back(vec4f);
    }
    boxConstruct();
    return res;
}

