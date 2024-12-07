#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>

class ImageProcessorNode : public rclcpp::Node
{
public:
    ImageProcessorNode()
    : Node("image_processor_node")
    {
        // 创建订阅者，订阅"/image"话题
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image", 10, std::bind(&ImageProcessorNode::image_callback, this, std::placeholders::_1));

        // 创建发布者，发布到"/start_detect"话题
        start_detect_publisher_ = this->create_publisher<std_msgs::msg::String>("/start_detect", 10);
    }

private:
    // 图像回调函数
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // 使用cv_bridge将ROS图像消息转换为OpenCV图像
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            // 处理图像
            process_image(cv_ptr->image);

        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    // 图像处理函数
    void process_image(cv::Mat& frame)
    {
        // 定义感兴趣区域 (ROI)
        cv::Rect roi(150, 230, 320, 210);  // 你可以调整这些参数
        cv::Mat roi_frame = frame(roi);

        // 转换为HSV颜色空间
        cv::Mat hsv;
        cv::cvtColor(roi_frame, hsv, cv::COLOR_BGR2HSV);

        // 定义白色的HSV范围
        cv::Scalar lower_white(0, 0, 180);   // 低范围
        cv::Scalar upper_white(180, 30, 255); // 高范围

        // 创建掩膜
        cv::Mat mask;
        cv::inRange(hsv, lower_white, upper_white, mask);

        // 形态学操作，消除噪点并填补空白
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
        cv::Mat eroded, dilated;
        cv::erode(mask, eroded, kernel);
        cv::dilate(eroded, dilated, kernel);

        // 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(dilated, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 遍历每个连通区域
        for (const auto& contour : contours)
        {
            double area = cv::contourArea(contour);
            RCLCPP_INFO(this->get_logger(), "Large white region detected with area: %f", area);
            if (area > 500)  // 如果区域面积大于500
            {
                //RCLCPP_INFO(this->get_logger(), "Large white region detected with area: %f", area);

                // 发布消息
                auto message = std::make_shared<std_msgs::msg::String>();
                message->data = "open";
                start_detect_publisher_->publish(*message);
                break;  // 可以根据需求做更多处理
            }
        }

    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr start_detect_publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessorNode>());
    rclcpp::shutdown();
    return 0;
}

