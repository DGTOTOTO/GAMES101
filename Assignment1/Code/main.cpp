#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) {
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1, -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle) {
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    // Identity(): 单位矩阵

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float theta = rotation_angle * MY_PI / 180;
    Eigen::Matrix4f zRotation({{std::cos(theta), -std::sin(theta), 0, 0},
                               {std::sin(theta), std::cos(theta), 0, 0},
                               {0, 0, 1, 0},
                               {0, 0, 0, 1}});

    return model * zRotation;
}

Eigen::Matrix4f perspective_to_orthographic(float zNear, float zFar) {
    // 从透视投影 -> 正交投影
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f perspectiveProjection(
        {{zNear, 0, 0, 0}, {0, zNear, 0, 0}, {0, 0, zNear + zFar, -zNear * zFar}, {0, 0, 1, 0}});
    Eigen::Matrix4f upsideMatrix({{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, -1, 0}, {0, 0, 0, 1}});

    return matrix * perspectiveProjection * upsideMatrix;
}

Eigen::Matrix4f get_final_matrix(float l, float r, float b, float t, float n, float f) {
    // 正交投影 -> 屏幕空间
    Eigen::Matrix4f scaleMatrix({{2 / (r - l), 0, 0, 0}, {0, 2 / (t - b), 0, 0}, {0, 0, 2 / (n - f), 0}, {0, 0, 0, 1}});
    Eigen::Matrix4f positionMatrx(
        {{1, 0, 0, -(l + r) / 2}, {0, 1, 0, -(b + t) / 2}, {0, 0, 1, -(n + f) / 2}, {0, 0, 0, 1}});

    return scaleMatrix * positionMatrx;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar) {
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

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    // 得到绕任意过原点的轴的旋转变换矩阵
    // 罗德里格斯旋转公式
    float theta = angle * MY_PI / 180;
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f N({{0, -axis[2], axis[1]}, {axis[2], 0, -axis[0]}, {-axis[1], axis[0], 0}});
    // 更好的写法：
    // N << 0, -axis.z(), axis.y(), axis.z(), 0, -axis.x(), -axis.y(), axis.x(),0;

    Eigen::Matrix3f R = std::cos(theta) * I + (1 - std::cos(theta)) * (axis * axis.transpose()) + std::sin(theta) * N;
    // Eigen::Matrix4f final(
    //     {{matrix(0), 0}, {matrix(1), 0}, {matrix(2), 0}, {matrix(3), 0}});
    // 不能用这个方法（3*3矩阵）初始化4*4矩阵，要用 block 方法（如下）

    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    rotation.block<3, 3>(0, 0) = R;
    return rotation;
}

// main 函数：模拟图形管线
int main(int argc, const char **argv) {
    float angle = 0; // 角度
    bool command_line = false;
    std::string filename = "output.png";

    // 示例命令行为 ./Rasterizer -r 20，“20” 是角度
    // 20 作为字符串先转化为浮点数 （stof 函数）
    // 运行结果储存在第四个命令行参数中
    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        } else
            return 0;
    }

    rst::rasterizer r(700, 700); // 这里定义了一个 rasterizer，同名构造函数一个是
                                 // w，一个是 h。应该是窗口大小

    Eigen::Vector3f eye_pos = {0, 0, 5}; // 应该是 摄影机变换

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}}; // 给定坐标（v0，v1， v2）

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    Eigen::Vector3f xRoation = {1, 0, 0};
    Eigen::Vector3f yRoation = {0, 1, 0};
    Eigen::Vector3f zRoation = {0, 0, 1};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;
    int axis = 0;

    // 处理命令行程序，可以直接返回
    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));                    // 应用模型变换
        r.set_view(get_view_matrix(eye_pos));                    // 应用相机变换
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50)); // 应用透视变换

        // 开始绘图
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        // 27: ESC 键

        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        if (axis == 1) {
            r.set_model(get_rotation(xRoation, angle));
        } else if (axis == 2) {
            r.set_model(get_rotation(yRoation, angle));
        } else if (axis == 3) {
            r.set_model(get_rotation(zRoation, angle));
        } else {
            r.set_model(get_model_matrix(angle));
        }
        r.set_view(get_view_matrix(eye_pos));                    // 相机视角变换
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50)); // 透视变换

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        switch (key) {
        case 'a':
            angle += 10;
            break;
        case 'd':
            angle -= 10;
            break;
        case 'x':
            axis = 1;
            break;
        case 'y':
            axis = 2;
            break;
        case 'z':
            axis = 3;
            break;
        default:
            break;
        }
    }

    return 0;
}