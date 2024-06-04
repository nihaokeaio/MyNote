#pragma once

#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

class Texture {
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Vec3f getColor(const Vec2f& pos) { return getColor(pos.x, pos.y); }

    Vec3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        if (u_img < 0)u_img = 0;
        if (v_img < 0)v_img = 0;
        if (u_img > width)u_img = width - 1;
        if (v_img > height)v_img = height - 1;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        auto val = Vec3f(color[0], color[1], color[2]);
        return Vec3f(color[0], color[1], color[2]);
    }

    Vec3f getColorBilinear(const Vec2f& pos) { return getColor(pos.x, pos.y); }

    Vec3f getColorBilinear(float u, float v)
    {
        auto h = [](Vec3f u, Vec3f v, float t)->Vec3f {
            return (1 - t) * u + t * v;
            };
        /*if (u < 0)u = 0;
        if (v < 0)v = 0;
        if (u >width)u = 0;
        if (v < 0)v = 0;*/
        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        if (u_img < 0)u_img = 0;
        if (v_img < 0)v_img = 0;
        if (u_img > width)u_img = width - 1;
        if (v_img > height)v_img = height - 1;

        Vec2i u00 = { (int)floor(u_img),(int)floor(v_img) };
        Vec2i u01 = { (int)floor(u_img),(int)ceil(v_img) };
        Vec2i u10 = { (int)ceil(u_img),(int)floor(v_img) };
        Vec2i u11 = { (int)ceil(u_img),(int)ceil(v_img) };

        auto val00 = image_data.at<cv::Vec3b>(u00.y, u00.x);
        auto color00 = Vec3f(val00[0], val00[1], val00[2]);

        auto val01 = image_data.at<cv::Vec3b>(u01.y, u01.x);
        auto color01 = Vec3f(val01[0], val01[1], val01[2]);

        auto val10 = image_data.at<cv::Vec3b>(u10.y, u10.x);
        auto color10 = Vec3f(val10[0], val10[1], val10[2]);

        auto val11 = image_data.at<cv::Vec3b>(u11.y, u11.x);
        auto color11 = Vec3f(val11[0], val11[1], val11[2]);

        float t0 = (u_img - u00.x) / (u10.x - u00.x);
        auto color0 = h(color00, color10, t0);

        float t1 = (u_img - u01.x) / (u11.x - u01.x);
        auto color1 = h(color01, color11, t0);

        float t = (v_img - u10.y) / (u11.y - u10.y);
        auto color = h(color0, color1, t);

        return color;
    }

};
