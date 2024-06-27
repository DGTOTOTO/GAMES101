#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // 加载图像
    cv::Mat image = cv::imread("test.jpg", cv::IMREAD_COLOR);
    if (image.empty()) {
        std::cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    // 创建窗口
    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
    // 显示图像
    cv::imshow("Display window", image);

    // 等待按键
    cv::waitKey(0);
    return 0;
}
