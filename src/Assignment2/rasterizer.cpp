// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <cmath>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector3f point = {x+0.5f, y+0.5f, 0.0f};

    // 返回叉乘 是否大于0
    auto fun_cross_product = [&](Eigen::Vector3f p0, Eigen::Vector3f p1, Eigen::Vector3f p2) -> bool
    {
        Eigen::Vector3f vector1 = p1 - p0;
        Eigen::Vector3f vector2 = p2 - p0;

        return (vector1.x() * vector2.y()) - (vector1.y() * vector2.x()) > 0;
    };

    bool rst0 = fun_cross_product(_v[0], _v[1], point);
    bool rst1 =fun_cross_product(_v[1], _v[2], point);
    bool rst2 = fun_cross_product(_v[2], _v[0], point);

    return rst0 == rst1 && rst1 == rst2;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
        //MSAA_rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    std::array<Vector4f, 3> v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    // If so, use the following code to get the interpolated z value.
    auto Vec_Min = [](Vector3f left, Vector3f right) -> Vector3f{
        return Vector3f(std::min(left.x(), right.x()), std::min(left.y(), right.y()),
                       std::min(left.z(), right.z()));
    };

    auto Vec_Max = [](Vector3f left, Vector3f right) -> Vector3f{
        return Vector3f(std::max(left.x(), right.x()), std::max(left.y(), right.y()),
                       std::max(left.z(), right.z()));
    };

    Eigen::Vector3f AABB_Min = Vec_Min(Vec_Min(t.v[0], t.v[1]), t.v[2]);
    Eigen::Vector3f AABB_Max = Vec_Max(Vec_Max(t.v[0], t.v[1]), t.v[2]);



    for(int32_t x = (AABB_Min.x()); x < (AABB_Max.x()); ++x)
    {
        for(int32_t y = (AABB_Min.y()); y < (AABB_Max.y()); ++y)
        {
            if(!insideTriangle(x, y, t.v))
            {
                continue;
            }

            auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
            float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            z_interpolated *= w_reciprocal;


            auto isLessThan = [](float a, float b) {
                    const float epsilon = 1e-5f;
                    return a + epsilon < b;
                };

            if(isLessThan(z_interpolated, depth_buf[get_index(x, y)]))
            {
                // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) 
                // if it should be painted.
                depth_buf[get_index(x, y)] = z_interpolated;

                Eigen::Vector3f point{(float)x, (float)y, 0.0f};
                Eigen::Vector3f color = t.getColor();

                set_pixel(point, color);
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }

    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        for(auto vec_it = MSAA_frame_buf.begin(); vec_it != MSAA_frame_buf.end(); ++vec_it)
        {        
            for (auto arr_it = vec_it->begin(); arr_it != vec_it->end(); ++arr_it)
            {
                *arr_it = Eigen::Vector3f(0.0f, 0.0f, 0.0f);  // 将每个 Eigen::Vector3f 初始化为 {0.0f, 0.0f, 0.0f}
            }
        }
    }

    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        for(auto vec_it = MSAA_depth_buf.begin(); vec_it != MSAA_depth_buf.end(); ++vec_it)
        {        
            for (auto arr_it = vec_it->begin(); arr_it != vec_it->end(); ++arr_it)
            {
                *arr_it = std::numeric_limits<float>::infinity();  // 将每个 Eigen::Vector3f 初始化为 {0.0f, 0.0f, 0.0f}
            }
        }
    }
    
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

    MSAA_frame_buf.resize(w*h);
    MSAA_depth_buf.resize(w*h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}


static bool MSAA_insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector3f point = {x, y, 0.0f};

    // 返回叉乘 是否大于0
    auto fun_cross_product = [&](Eigen::Vector3f p0, Eigen::Vector3f p1, Eigen::Vector3f p2) -> bool
    {
        Eigen::Vector3f vector1 = p1 - p0;
        Eigen::Vector3f vector2 = p2 - p0;

        return (vector1.x() * vector2.y()) - (vector1.y() * vector2.x()) > 0;
    };

    bool rst0 = fun_cross_product(_v[0], _v[1], point);
    bool rst1 =fun_cross_product(_v[1], _v[2], point);
    bool rst2 = fun_cross_product(_v[2], _v[0], point);

    return rst0 == rst1 && rst1 == rst2;
}

void rst::rasterizer::MSAA_rasterize_triangle(const Triangle& t) {
    std::array<Vector4f, 3> v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    // If so, use the following code to get the interpolated z value.
    auto Vec_Min = [](Vector3f left, Vector3f right) -> Vector3f{
        return Vector3f(std::min(left.x(), right.x()), std::min(left.y(), right.y()),
                       std::min(left.z(), right.z()));
    };

    auto Vec_Max = [](Vector3f left, Vector3f right) -> Vector3f{
        return Vector3f(std::max(left.x(), right.x()), std::max(left.y(), right.y()),
                       std::max(left.z(), right.z()));
    };

    Eigen::Vector3f AABB_Min = Vec_Min(Vec_Min(t.v[0], t.v[1]), t.v[2]);
    Eigen::Vector3f AABB_Max = Vec_Max(Vec_Max(t.v[0], t.v[1]), t.v[2]);

    const int32_t Frequence = 2;
    const int32_t MSAA_frequnce = Frequence*Frequence;
    const float MSAA_center = 1.0/MSAA_frequnce;

    auto isLessThan = [](float a, float b) {
            const float epsilon = 1e-5f;
            return a + epsilon < b;
        };
    
    for(int32_t x = (AABB_Min.x()); x < (AABB_Max.x()); ++x)
    {
        for(int32_t y = (AABB_Min.y()); y < (AABB_Max.y()); ++y)
        {
            int32_t index = 0;
            for(float msaa_x = x; isLessThan(msaa_x, x+1.0); msaa_x+=(1.0/Frequence))
            for(float msaa_y = y; isLessThan(msaa_y, y+1.0); msaa_y+=(1.0/Frequence))
            {
                float pixel_center_x = msaa_x+MSAA_center;
                float pixel_center_y = msaa_y+MSAA_center;


                if(!MSAA_insideTriangle(pixel_center_x, pixel_center_y, t.v))
                {
                    continue;
                }

                auto[alpha, beta, gamma] = computeBarycentric2D(pixel_center_x, pixel_center_y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
            
                if(isLessThan(z_interpolated, MSAA_depth_buf[get_index(x,y)][index]))
                {
                    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) 
                    // if it should be painted.
                    MSAA_depth_buf[get_index(x,y)][index] = z_interpolated;

                    Eigen::Vector3f point{(float)x, (float)y, 0.0f};
                    Eigen::Vector3f color = t.getColor();

                    MSAA_frame_buf[get_index(x,y)][index] = color/MSAA_frequnce;
                }

                ++index;
            }
        }
    }

    for(int32_t i = 0; i < MSAA_frame_buf.size(); ++i)
    {
        Eigen::Vector3f color = {0, 0, 0};
        for(int32_t j = 0; j < MSAA_frame_buf[i].size(); ++j)
        {
            color = color + MSAA_frame_buf[i][j];
        }

        frame_buf[i] = color;
    }
}

// clang-format on