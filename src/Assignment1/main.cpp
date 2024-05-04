#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    Eigen::Matrix4f rotate;
    float radian = rotation_angle/180 * std::acos(-1);
    rotate << std::cos(radian), -std::sin(radian), 0, 0
            , std::sin(radian), std::cos(radian), 0, 0
            , 0, 0, 1, 0
            , 0, 0, 0, 1;

    model = rotate * model;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
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
    orthographic << 2/width, 0, 0, 0
                    , 0, 2/height, 0, 0
                    , 0, 0, 2/(zFar-zNear), -(zFar+zNear)/2
                    , 0, 0, 0, 1;
                    
    Eigen::Matrix4f transform;//正交投影需要先移动到原点
    transform << 1, 0, 0, 0
                , 0, 1, 0, 0
                , 0, 0, 1, -(zNear+zFar)/2
                , 0, 0, 0, 1;

    projection = orthographic * transform * perspective * projection;
    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    float radian = angle/180 * std::acos(-1);
    float nx = axis[0];
    float ny = axis[1];
    float nz = axis[2];

    // 1,0,0绕axis旋转得到的向量
    Vector3f p1 = {std::cos(radian) + (1-std::cos(radian))*nx*nx, (1-std::cos(radian))*nx*ny-std::sin(radian)*nz, (1-std::cos(radian))*nx*ny+std::sin(radian)*ny};
    // 0,1,0绕axis旋转得到的向量
    Vector3f p2 = {(1-std::cos(radian))*ny*nx + std::sin(radian)*nz, std::cos(radian)+(1-std::cos(radian))*ny*ny,(1-std::cos(radian))*ny*nz - std::sin(radian)*nx};
    // 0,0,1绕axis旋转得到的向量
    Vector3d p3 = {(1-std::cos(radian))*nx*nz-std::sin(radian)*ny, (1-std::cos(radian))*ny*nz+std::sin(radian)*nx, std::cos(radian)+(1-std::cos(radian))*nz*nz};

    // 先求(1,0,0)绕axis旋转的Q矩阵，Q的转置/逆，就是我们需要的绕axis的旋转矩阵了
    Eigen::Matrix4f Q;
    Q << p1[0], p2[0], p3[0], 0
        , p1[1], p2[1], p3[1], 0
        , p1[2], p2[2], p3[2], 0
        , 0, 0, 0, 1;

    Eigen::Matrix4f M;
    M << 1, 0, 0, 0
        , 0, 1, 0, 0
        , 0, 0, 1, 2
        , 0, 0, 0, 1;

    Eigen::Matrix4f Qt = Q.inverse();

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    projection = M.inverse() * Qt * M * projection;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};
    Eigen::Vector3f axis = {0,1.414/2, -1.414/2};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));
        r.set_model(get_rotation(axis, angle));

        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << axis << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }

        if (key == 'x') {
            axis[0] += 1;
        }
        else if (key == 'y') {
            axis[1] += 1;
        }
        else if(key == 'z')
        {
            axis[2] += 1;
        }
    }

    return 0;
}
