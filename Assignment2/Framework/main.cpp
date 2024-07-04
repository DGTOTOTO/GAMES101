// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"

constexpr double MY_PI = 3.1415926;

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

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    return model;
}



Eigen::Matrix4f perspective_to_orthographic(float zNear, float zFar) {
  // 从透视投影 -> 正交投影
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f perspectiveProjection({{zNear, 0, 0, 0},
                                         {0, zNear, 0, 0},
                                         {0, 0, zNear + zFar, -zNear * zFar},
                                         {0, 0, 1, 0}});
  Eigen::Matrix4f upsideMatrix(
      {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, -1, 0}, {0, 0, 0, 1}});

  return matrix * perspectiveProjection * upsideMatrix;
}

Eigen::Matrix4f get_final_matrix(float l, float r, float b, float t, float n,
                                 float f) {
  // 正交投影 -> 屏幕空间
  Eigen::Matrix4f scaleMatrix({{2 / (r - l), 0, 0, 0},
                               {0, 2 / (t - b), 0, 0},
                               {0, 0, 2 / (n - f), 0},
                               {0, 0, 0, 1}});
  Eigen::Matrix4f positionMatrx({{1, 0, 0, -(l + r) / 2},
                                 {0, 1, 0, -(b + t) / 2},
                                 {0, 0, 1, -(n + f) / 2},
                                 {0, 0, 0, 1}});

  return scaleMatrix * positionMatrx;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar) {
    // TODO: Copy-paste your implementation from the previous assignment.
      // 垂直可视角（fov），近平面宽比，近点，远点
  // Students will implement this function

  Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

  // TODO: Implement this function
  // Create the projection matrix for the given parameters.
  // Then return it.

  projection = perspective_to_orthographic(zNear, zFar); // 转换为正交投影

  float angle = eye_fov / 180.0 * MY_PI; // 转换成弧度

  // 从可视角、近平面宽比到小长方体（l, r, b, t, f, n）
  float t = zNear * std::tan(angle / 2);
  float r = t * aspect_ratio;
  float l = -r;
  float b = -t;
  projection = get_final_matrix(l, r, b, t, zNear, zFar) * projection;

  return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0; // 旋转角度
    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    // 初始化栅格化器
    rst::rasterizer r(700, 700);

    // 初始化摄像机
    Eigen::Vector3f eye_pos = {0,0,5};

    // 初始化位置
    std::vector<Eigen::Vector3f> pos
            {
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2},
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5}
            };

    // 初始化索引
    std::vector<Eigen::Vector3i> ind
            {
                    {0, 1, 2},
                    {3, 4, 5}
            };

    // 初始化颜色
    std::vector<Eigen::Vector3f> cols
            {
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0}
            };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);  // 绘制
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        // ⚠️ 主要调用的函数
        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }

    return 0;
}
// clang-format on