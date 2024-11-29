#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class HSVThresholdingNode : public rclcpp::Node
{
public:
    HSVThresholdingNode() : Node("hsv_thresholding_node")
    {
        // 初始化订阅者
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&HSVThresholdingNode::image_callback, this, std::placeholders::_1));

        // 初始化OpenCV窗口
        cv::namedWindow("HSV Thresholding", cv::WINDOW_NORMAL);

        // 创建滑块，初始化HSV阈值
        cv::createTrackbar("H Min", "HSV Thresholding", &h_min_, 179);
        cv::createTrackbar("H Max", "HSV Thresholding", &h_max_, 179);
        cv::createTrackbar("S Min", "HSV Thresholding", &s_min_, 255);
        cv::createTrackbar("S Max", "HSV Thresholding", &s_max_, 255);
        cv::createTrackbar("V Min", "HSV Thresholding", &v_min_, 255);
        cv::createTrackbar("V Max", "HSV Thresholding", &v_max_, 255);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // 将ROS图像消息转换为OpenCV格式
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            // 将图像转换为HSV格式
            cv::Mat hsv_image;
            cv::cvtColor(frame, hsv_image, cv::COLOR_BGR2HSV);

            // 定义HSV阈值范围
            cv::Scalar lower_bound(h_min_, s_min_, v_min_);
            cv::Scalar upper_bound(h_max_, s_max_, v_max_);

            // 根据HSV阈值进行图像分割
            cv::Mat thresholded_image;
            cv::inRange(hsv_image, lower_bound, upper_bound, thresholded_image);

            // 显示阈值处理后的图像
            cv::imshow("HSV Thresholding", thresholded_image);
            cv::waitKey(1);  // 等待并更新窗口
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not convert image: %s", e.what());
        }
    }

    // 订阅图像数据
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

    // 滑块的值：HSV阈值的最小值和最大值
    int h_min_ = 0, h_max_ = 179;
    int s_min_ = 0, s_max_ = 255;
    int v_min_ = 0, v_max_ = 255;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HSVThresholdingNode>());
    rclcpp::shutdown();
    return 0;
}

