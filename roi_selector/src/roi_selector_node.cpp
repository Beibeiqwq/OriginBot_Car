#include <unordered_map>
#include <iostream>
#include <functional>
#include <future>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

class RoiSelectorNode : public rclcpp::Node
{
public:
    RoiSelectorNode() : Node("roi_selector_node")
    {   
        //using namespace std::placeholders;
        // 创建图像订阅器
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg)
            {
                image_callback(msg);
            });

        RCLCPP_INFO(this->get_logger(), "RoiSelectorNode started.");
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    cv::Rect roi_;
    bool is_drawing_ = false;
    sensor_msgs::msg::Image::SharedPtr latest_msg_;  // 存储最新的图像消息

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // 更新最新的图像消息
            latest_msg_ = msg;
            // 将 ROS 图像消息转换为 OpenCV 图像
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // 显示图像
            if (!frame.empty())
            {
                cv::imshow("Image", frame);
                cv::setMouseCallback("Image", &RoiSelectorNode::mouse_callback, this);
                cv::waitKey(1);  // 等待键盘事件

                // 如果框选了 ROI，输出坐标
                if (roi_.area() > 0)
                {
                    // 输出四个顶点坐标
                    RCLCPP_INFO(this->get_logger(), "ROI Top-Left: (%d, %d)", roi_.x, roi_.y);
                    RCLCPP_INFO(this->get_logger(), "ROI Top-Right: (%d, %d)", roi_.x + roi_.width, roi_.y);
                    RCLCPP_INFO(this->get_logger(), "ROI Bottom-Left: (%d, %d)", roi_.x, roi_.y + roi_.height);
                    RCLCPP_INFO(this->get_logger(), "ROI Bottom-Right: (%d, %d)", roi_.x + roi_.width, roi_.y + roi_.height);
                }
            }
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    // 鼠标回调函数，用于绘制和选择 ROI
    static void mouse_callback(int event, int x, int y, int flags, void *userdata)
    {
        RoiSelectorNode *self = static_cast<RoiSelectorNode *>(userdata);

        if (self->latest_msg_ == nullptr)  // 如果没有收到图像消息
            return;

        // 获取当前图像
        cv::Mat frame = cv_bridge::toCvCopy(self->latest_msg_, "bgr8")->image;

        if (event == cv::EVENT_LBUTTONDOWN)
        {
            self->is_drawing_ = true;
            self->roi_ = cv::Rect(x, y, 0, 0);
        }
        else if (event == cv::EVENT_MOUSEMOVE && self->is_drawing_)
        {
            self->roi_.width = x - self->roi_.x;
            self->roi_.height = y - self->roi_.y;
        }
        else if (event == cv::EVENT_LBUTTONUP)
        {
            self->is_drawing_ = false;
            // 在图像中绘制最终的矩形 ROI
            cv::rectangle(frame, self->roi_, cv::Scalar(0, 255, 0), 2);
            cv::imshow("Image", frame);
            cv::waitKey(1);

            // 输出四个顶点坐标
            RCLCPP_INFO(self->get_logger(), "ROI Top-Left: (%d, %d)", self->roi_.x, self->roi_.y);
            RCLCPP_INFO(self->get_logger(), "ROI Top-Right: (%d, %d)", self->roi_.x + self->roi_.width, self->roi_.y);
            RCLCPP_INFO(self->get_logger(), "ROI Bottom-Left: (%d, %d)", self->roi_.x, self->roi_.y + self->roi_.height);
            RCLCPP_INFO(self->get_logger(), "ROI Bottom-Right: (%d, %d)", self->roi_.x + self->roi_.width, self->roi_.y + self->roi_.height);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoiSelectorNode>());
    rclcpp::shutdown();
    return 0;
}
