#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ShapeRecognitionNode : public rclcpp::Node
{
public:
    ShapeRecognitionNode() : Node("shape_recognition_node")
    {
        // 初始化订阅器，订阅图像数据
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&ShapeRecognitionNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Shape Recognition Node has been started.");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // 将ROS2图像消息转换为OpenCV图像
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // 进行图像处理
            process_image(frame);

        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not convert image: %s", e.what());
        }
    }

    void process_image(cv::Mat &frame)
    {
        cv::Mat gray, blurred, edges;

        // 将图像转换为灰度图
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1.5);

        // 使用Canny边缘检测
        cv::Canny(blurred, edges, 100, 200);

        // 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        for (const auto &contour : contours)
        {
            // 近似轮廓为多边形
            std::vector<cv::Point> approx;
            cv::approxPolyDP(contour, approx, 5, true);

            // 获取轮廓的面积
            double area = cv::contourArea(approx);
            if (area < 100) continue; // 忽略小的轮廓

            // 判断不同形状
            if (approx.size() == 4)
            {
                // 识别矩形：灰色矩形 
                cv::Rect bounding_rect = cv::boundingRect(approx);
                cv::rectangle(frame, bounding_rect, cv::Scalar(128, 128, 128), 2);
                RCLCPP_INFO(this->get_logger(), "Detected Gray Rectangle.");
            }
            else if (approx.size() > 5)
            {
                // 识别椭圆形：绿色椭圆
                cv::RotatedRect ellipse = cv::fitEllipse(approx);
                cv::ellipse(frame, ellipse, cv::Scalar(0, 255, 0), 2);
                RCLCPP_INFO(this->get_logger(), "Detected Green Ellipse.");
            }
            else if (approx.size() == 4 && is_diamond(approx))
            {
                // 识别菱形：蓝色菱形
                cv::polylines(frame, approx, true, cv::Scalar(255, 0, 0), 2);
                RCLCPP_INFO(this->get_logger(), "Detected Blue Diamond.");
            }
        }

        // 显示结果
        cv::imshow("Detected Shapes", frame);
        cv::waitKey(1); // 防止显示窗口卡死
    }

    bool is_diamond(const std::vector<cv::Point> &approx)
    {
        // 判断是否为菱形，可以根据角度或长宽比来判断
        cv::Point2f p1 = approx[0];
        cv::Point2f p2 = approx[1];
        cv::Point2f p3 = approx[2];
        cv::Point2f p4 = approx[3];

        double angle1 = std::abs(cv::fastAtan2(p2.y - p1.y, p2.x - p1.x) - cv::fastAtan2(p3.y - p2.y, p3.x - p2.x));
        double angle2 = std::abs(cv::fastAtan2(p3.y - p2.y, p3.x - p2.x) - cv::fastAtan2(p4.y - p3.y, p4.x - p3.x));
        return angle1 > 60 && angle2 > 60;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ShapeRecognitionNode>());
    rclcpp::shutdown();
    return 0;
}
