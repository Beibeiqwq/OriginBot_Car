#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <unordered_map>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <functional>
using namespace cv;
using namespace std;

class TrafficLightDetectionNode : public rclcpp::Node
{
public:
    TrafficLightDetectionNode(const rclcpp::NodeOptions &options)
        : Node("traffic_light_detection_node", options)
    {
        // 读取参数
        this->declare_parameter("lower_red_hue", 0);
        this->declare_parameter("upper_red_hue", 10);
        this->declare_parameter("lower_red_saturation", 120);
        this->declare_parameter("upper_red_saturation", 255);
        this->declare_parameter("lower_red_value", 100);
        this->declare_parameter("upper_red_value", 255);

        this->declare_parameter("lower_green_hue", 60);
        this->declare_parameter("upper_green_hue", 179);
        this->declare_parameter("lower_green_saturation", 150);
        this->declare_parameter("upper_green_saturation", 255);
        this->declare_parameter("lower_green_value", 60);
        this->declare_parameter("upper_green_value", 255);

        // 获取参数值
        int lower_red_hue = this->get_parameter("lower_red_hue").as_int();
        int upper_red_hue = this->get_parameter("upper_red_hue").as_int();
        int lower_red_saturation = this->get_parameter("lower_red_saturation").as_int();
        int upper_red_saturation = this->get_parameter("upper_red_saturation").as_int();
        int lower_red_value = this->get_parameter("lower_red_value").as_int();
        int upper_red_value = this->get_parameter("upper_red_value").as_int();

        int lower_green_hue = this->get_parameter("lower_green_hue").as_int();
        int upper_green_hue = this->get_parameter("upper_green_hue").as_int();
        int lower_green_saturation = this->get_parameter("lower_green_saturation").as_int();
        int upper_green_saturation = this->get_parameter("upper_green_saturation").as_int();
        int lower_green_value = this->get_parameter("lower_green_value").as_int();
        int upper_green_value = this->get_parameter("upper_green_value").as_int();

        // 设置红色和绿色的HSV范围
        lower_red_ = Scalar(lower_red_hue, lower_red_saturation, lower_red_value);
        upper_red_ = Scalar(upper_red_hue, upper_red_saturation, upper_red_value);

        lower_green_ = Scalar(lower_green_hue, lower_green_saturation, lower_green_value);
        upper_green_ = Scalar(upper_green_hue, upper_green_saturation, upper_green_value);

        // 订阅"/image_raw"话题
        // image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        //     "/image_raw", 10,
        //     std::bind(&TrafficLightDetectionNode::image_callback, this, std::placeholders::_1));
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/robotvoice", 10,
            std::bind(&TrafficLightDetectionNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Traffic Light Detection Node Initialized.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // 将ROS图像消息转换为OpenCV图像
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            // 调用红绿灯检测函数
            detect_traffic_light(cv_ptr->image);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error converting image: %s", e.what());
        }
    }

    void detect_traffic_light(const Mat &frame)
    {
        // 转换为HSV色彩空间
        Mat hsv_image;
        cvtColor(frame, hsv_image, COLOR_BGR2HSV);

        // 根据颜色范围进行掩码操作
        Mat mask_red, mask_green;
        inRange(hsv_image, lower_red_, upper_red_, mask_red);
        inRange(hsv_image, lower_green_, upper_green_, mask_green);

        // 检测红绿灯状态
        vector<Vec3f> circles_red, circles_green;

        // 检测红灯圆形
        HoughCircles(mask_red, circles_red, HOUGH_GRADIENT, 1, frame.rows / 8, 100, 30, 10, 200);

        // 检测绿灯圆形
        HoughCircles(mask_green, circles_green, HOUGH_GRADIENT, 1, frame.rows / 8, 100, 30, 10, 200);
        /*
        1：累加器分辨率。通常设置为1，即保持图像分辨率不变。
        frame.rows / 8：最小圆心间距（最小圆之间的距离）。设置为图像高度的 1/8，意味着我们期望圆之间的距离不会小于该值。
        100：累加器阈值，表示一个圆被检测到所需的最小投票数。值越小，检测的圆越多，可能包括噪声；值越大，检测出的圆越少，但可能更准确。
        30：最小圆的半径，表示检测时最小圆的大小。根据红绿灯的尺寸可以调整。
        10：最大圆的半径，表示检测时最大圆的大小。同样根据需要调整。
        200：最大圆的半径，表示检测时最大圆的半径。根据红绿灯的实际尺寸来调整
        */
        if (!circles_red.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Red light detected.");
        }
        else if (!circles_green.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Green light detected.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No traffic light detected.");
        }
    }

    // 红绿灯的HSV颜色范围
    Scalar lower_red_;
    Scalar upper_red_;
    Scalar lower_green_;
    Scalar upper_green_;

    // 图像订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 创建节点选项，加载参数文件
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"lower_red_hue", 0},          // 红色HSV范围
        {"upper_red_hue", 10},         // 红色HSV范围
        {"lower_red_saturation", 120}, // 红色HSV范围
        {"upper_red_saturation", 255}, // 红色HSV范围
        {"lower_red_value", 100},      // 红色HSV范围
        {"upper_red_value", 255},      // 红色HSV范围

        {"lower_green_hue", 60},         // 绿色HSV范围
        {"upper_green_hue", 179},        // 绿色HSV范围
        {"lower_green_saturation", 150}, // 绿色HSV范围
        {"upper_green_saturation", 255}, // 绿色HSV范围
        {"lower_green_value", 60},       // 绿色HSV范围
        {"upper_green_value", 255}       // 绿色HSV范围
    });

    // 创建节点实例
    auto node = std::make_shared<TrafficLightDetectionNode>(options);

    // 运行节点
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
