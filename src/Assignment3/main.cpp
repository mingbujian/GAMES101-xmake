#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Use the same projection matrix from the previous assignments
     zNear = -zNear;
     zFar = -zFar;
    
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f perspective;//透视投影
    perspective << zNear, 0, 0, 0
                , 0, zNear, 0, 0
                , 0, 0, zNear + zFar, -1*zNear*zFar
                , 0, 0, 1, 0;

    float height = 2*std::tan(eye_fov/2)*std::fabs(zNear);
    float width = aspect_ratio * height;

    Eigen::Matrix4f orthographic;//正交投影

    Eigen::Matrix4f transform;//正交投影需要先移动到原点
    transform << 1, 0, 0, 0
        , 0, 1, 0, 0
        , 0, 0, 1, -(zNear + zFar) / 2
        , 0, 0, 0, 1;

    Eigen::Matrix4f Scale;  // 缩放[-1,1]，y[-1,1]，z[-1,1] NDC空间(归一化的设备坐标)
    Scale << 2/width, 0, 0, 0
                    , 0, 2/height, 0, 0
                    , 0, 0, 2/(zFar-zNear), -(zFar+zNear)/2
                    , 0, 0, 0, 1;
                    
    orthographic = Scale * transform;
    projection = orthographic * perspective * projection;
    return projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        float u = std::clamp(payload.tex_coords.x(), 0.0f, 1.0f);
        float v = std::clamp(payload.tex_coords.y(), 0.f, 1.0f);

        return_color = payload.texture->getColor(u, v);
    }

    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        // 
        
        // 逐元素相乘，也就是改变各个维度的值不一样
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

        // 光源到物体的距离的平方
        float r2 = std::pow(light.position.x() - point.x(), 2)
            + std::pow(light.position.y() - point.y(), 2)
            + std::pow(light.position.z() - point.z(), 2);

        Eigen::Vector3f diffuse = kd.cwiseProduct(light.intensity / r2) * std::max(0.0f, normal.dot((light.position - point).normalized()));

        Eigen::Vector3f half = ((light.position - point) + (eye_pos - point)).normalized();
        Eigen::Vector3f specular = ks.cwiseProduct(light.intensity / r2) * std::pow(std::max(0.0f, normal.dot(half)), p);

        result_color += (ambient + diffuse + specular);
    }

    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        // 
        // 逐元素相乘，也就是改变各个维度的值不一样
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

        // 光源到物体的距离的平方
        float r2 = std::pow(light.position.x() - point.x(), 2)
                    + std::pow(light.position.y() - point.y(), 2)
                    + std::pow(light.position.z() - point.z(), 2);

        Eigen::Vector3f diffuse = kd.cwiseProduct(light.intensity / r2) * std::max(0.0f, normal.dot((light.position - point).normalized()));
        
        Eigen::Vector3f half = ((light.position - point) + (eye_pos - point)).normalized();
        Eigen::Vector3f specular = ks.cwiseProduct(light.intensity / r2) * std::pow(std::max(0.0f, normal.dot(half)), p);

        // 测试各种光的效果
        //ambient = { 0,0,0 };
        //diffuse = { 0, 0, 0 };
        //specular = { 0,0,0 };

        result_color += (ambient + diffuse + specular);
    }

    return result_color * 255.f;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)

    // 切线向量（Tangent Vector，T）：这个向量沿着表面的一条边延伸，通常与纹理坐标的U轴对齐。
    Eigen::Vector3f t = { normal.x() * normal.y() / std::sqrt(normal.x() * normal.x() + normal.z() * normal.z())
        , std::sqrt(normal.x() * normal.x() + normal.z() * normal.z())
        , normal.z() * normal.y() / std::sqrt(normal.x() * normal.x() + normal.z() * normal.z()) };

    // 法线向量（Normal Vector，N）：这个向量垂直于表面，表示表面的法线方向。
    auto n = normal;

    // 副切线向量（Bitangent Vector，B）：这个向量沿着表面的另一条边延伸，通常与纹理坐标的V轴对齐，有时也被称为Binormal向量。
    Eigen::Vector3f b = normal.cross(n);

    Eigen::Matrix3f TBN;
    TBN << t.x(), b.x(), n.x()
        , t.y(), b.y(), n.y()
        , t.z(), b.z(), n.z();

    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        float u = std::clamp(payload.tex_coords.x(), 0.0f, 1.0f);
        float v = std::clamp(payload.tex_coords.y(), 0.f, 1.0f);

        auto h_fun = [&payload](float u, float v) {
            float u0 = std::clamp(u, 0.0f, 1.0f);
            float v0 = std::clamp(v, 0.f, 1.0f);
            return payload.texture->getColor(u0, v0);
            };

        auto height = payload.texture->height;
        auto width = payload.texture->width;

        auto dU = kh * kn * (h_fun(u + 1.f / width, v).norm() - h_fun(u, v).norm());
        auto dV = kh * kn * (h_fun(u, v + 1.f / height).norm() - h_fun(u, v).norm());

        Eigen::Vector3f ln = { -dU,-dV,1.0f };

        point = point + kn * n * h_fun(u, v).norm();// 把模型上的点真实改变
        normal = TBN * ln;
    }


    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

        // 光源到物体的距离的平方
        float r2 = std::pow(light.position.x() - point.x(), 2)
            + std::pow(light.position.y() - point.y(), 2)
            + std::pow(light.position.z() - point.z(), 2);

        Eigen::Vector3f diffuse = kd.cwiseProduct(light.intensity / r2) * std::max(0.0f, normal.dot((light.position - point).normalized()));

        Eigen::Vector3f half = ((light.position - point) + (eye_pos - point)).normalized();
        Eigen::Vector3f specular = ks.cwiseProduct(light.intensity / r2) * std::pow(std::max(0.0f, normal.dot(half)), p);

        // 测试各种光的效果
        //ambient = { 0,0,0 };
        //diffuse = { 0, 0, 0 };
        //specular = { 0,0,0 };

        result_color += (ambient + diffuse + specular);
    }

    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;


    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)

    // 切线向量（Tangent Vector，T）：这个向量沿着表面的一条边延伸，通常与纹理坐标的U轴对齐。
    Eigen::Vector3f t = {normal.x() * normal.y()/ std::sqrt(normal.x()*normal.x() + normal.z()*normal.z())
        , std::sqrt(normal.x() * normal.x() + normal.z() * normal.z())
        , normal.z()* normal.y() / std::sqrt(normal.x() * normal.x() + normal.z() * normal.z()) };

    // 法线向量（Normal Vector，N）：这个向量垂直于表面，表示表面的法线方向。
    auto n = normal;

    // 副切线向量（Bitangent Vector，B）：这个向量沿着表面的另一条边延伸，通常与纹理坐标的V轴对齐，有时也被称为Binormal向量。
    Eigen::Vector3f b = normal.cross(n);

    Eigen::Matrix3f TBN;
    TBN << t.x(), b.x(), n.x()
        , t.y(), b.y(), n.y()
        , t.z(), b.z(), n.z();

    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        float u = std::clamp(payload.tex_coords.x(), 0.0f, 1.0f);
        float v = std::clamp(payload.tex_coords.y(), 0.f, 1.0f);

        auto h_fun = [&payload](float u, float v) {
            float u0 = std::clamp(u, 0.0f, 1.0f);
            float v0 = std::clamp(v, 0.f, 1.0f);
            return payload.texture->getColor(u0, v0);
            };

        auto height = payload.texture->height;
        auto width = payload.texture->width;

        auto dU = kh * kn * (h_fun(u + 1.f/width, v).norm() - h_fun(u, v).norm());
        auto dV = kh * kn * (h_fun(u, v + 1.f/height).norm() - h_fun(u, v).norm());
        
        Eigen::Vector3f ln = { -dU,-dV,1.0f };

        normal = TBN * ln;
    }

    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = normal;

    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../../models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("../../models/spot/spot_triangulated_good.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    auto texture_path = "hmap.jpg";
    r.set_texture(Texture(obj_path + texture_path));

    //std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;
    //std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = texture_fragment_shader;
    //std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = bump_fragment_shader;
    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = displacement_fragment_shader;


    Eigen::Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader);

    int key = 0;
    int frame_count = 0;


    auto last_time = std::chrono::steady_clock::now();
    int32_t count = 0;

    while(key != 27)
    {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_time);

        if (elapsed_time.count() > 3)
        {
            count++;
            last_time = current_time;
        }

        const int32_t Number = 5;
        if(0 ==count % Number)
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;

            auto texture_path = "hmap.jpg";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (1 == count % Number)
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (2 == count % Number)
        { 
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;

            auto texture_path = "hmap.jpg";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (3 == count % Number)
        { 
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;

            auto texture_path = "hmap.jpg";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (4 == count % Number)
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;

            auto texture_path = "hmap.jpg";
            r.set_texture(Texture(obj_path + texture_path));
        }

        r.set_fragment_shader(active_shader);


        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }

    }
    return 0;
}
