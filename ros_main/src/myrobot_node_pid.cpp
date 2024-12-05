#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <functional>
#include <memory>
#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace std::chrono_literals;

enum MyRobotState
{
    STATE_WAIT,      // 等待状态
    STATE_WAITING,   // 识别红绿灯
    STATE_START,     // 开始前进
    STATE_FIND,      // 找棋子
    STATE_FIND_FLAG, // 找到棋子
    STATE_FIND_END,  // 播报结束
    STATE_FINDBALL,  // 寻找小球

};
enum ImageState
{
    IMAGE_STATE_FOLLOW_LINE,

};
ImageState ImageState_ = IMAGE_STATE_FOLLOW_LINE;

class MyRobotNodePid : public rclcpp::Node
{
public:
    MyRobotNodePid() : Node("my_robot_node_pid"), state_(STATE_WAIT)
    {
        op_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/op/image_raw", 10, std::bind(&MyRobotNodePid::opCallback, this, std::placeholders::_1));

        op_road_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/op/road_image_raw", 10, std::bind(&MyRobotNodePid::oproadCallback, this, std::placeholders::_1));
        // pose_sub_= this->create_subscription<nav_msgs::msg::Odometry>(
        //"odom", 10, std::bind(&MyRobotNodePid::pose_callback, this, std::placeholders::_1));

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        voice_pub_ = this->create_publisher<std_msgs::msg::String>("/voice", 10);
        pid_pub_ = this->create_publisher<std_msgs::msg::Float64>("/pid", 10);

        timer_ = this->create_wall_timer(std::chrono::duration<double, std::milli>(dt_ * 1000), std::bind(&MyRobotNodePid::timercallback, this));
    }

private:
    int count = 0;
    MyRobotState state_;
    double dt_ = 0.1;
    void timercallback()
    {
        switch (state_)
        {
        case STATE_WAIT:
            state_ = STATE_WAITING;
            break;
        case STATE_WAITING:
            // if (isGreen(image) == true)
            //{
            //  识别红绿灯
            state_ = STATE_START;
            //}
            break;
        case STATE_START:
            break;
        case STATE_FIND:
            // 找棋子
            break;
        case STATE_FIND_FLAG:
            // 找到棋子
            // 播报
            break;
        case STATE_FIND_END:
            count++;
            if (count == 3)
            {
                state_ = STATE_FINDBALL;
            }
            else
            {
                state_ = STATE_FIND;
            }
            break;
        case STATE_FINDBALL:
            // 寻找小球
            break;
        default:
            break;
        }
    }
    // 发布
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_; // 速度命令发布者
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pid_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr voice_pub_;
    // 订阅
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr op_road_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr op_sub_;
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_; // 控制循环定时器
    // 回调函数
    void oproadCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void opCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    // 功能函数
    void speak(std::string &voice);
    bool isAboveThreshold(cv::Mat &image, int threshold);
};
void MyRobotNodePid::opCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "CV Bridge Error: %s", e.what());
        return;
    }
    cv::Mat image = cv_ptr->image;
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    GaussianBlur(gray, gray, cv::Size(5, 5), 0);
    image = cv_ptr->image;

    if (ImageState_ == IMAGE_STATE_FOLLOW_LINE)
    {
        // * 动态阈值二值化
        cv::Mat gray;
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
        // * 高斯滤波
        GaussianBlur(gray, gray, cv::Size(5, 5), 0);
        // * 使用大津法进行二值化
        cv::Mat binary;
        cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

        // * 显示二值化结果
#ifdef Desktop
        cv::imshow("Binary Image", binary);
        cv::waitKey(1); // 这一步很重要，保持 OpenCV 窗口更新
#endif

#ifdef FLAG
        // * 旗帜判断
        cv::Mat hsv_flag;
        cvtColor(cv_ptr->image, hsv_flag, COLOR_BGR2HSV);

        // * 使用黄色范围进行颜色过滤
        Mat yellow_mask;
        inRange(hsv_flag, CvThreshold.lower_yellow_, CvThreshold.upper_yellow_, yellow_mask);

        // * 查找黄色区域的轮廓
        vector<vector<Point>> contours;
        findContours(yellow_mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        int min_block_width = 20;
        int min_block_height = 20;
        int gray_pixel_threshold = 400;
        int surrounding_area_margin = 3;
    }
    // * 遍历找到的轮廓
    for (size_t i = 0; i < contours.size(); i++)
    {
        // 获取矩形边界框
        Rect bounding_rect = boundingRect(contours[i]);

        // 过滤掉小的黄色色块
        if (bounding_rect.width < min_block_width || bounding_rect.height < min_block_height)
        {
            continue; // 如果黄色色块太小，则跳过
        }

        // 在图像上画出黄色区域的边界框
        rectangle(cv_ptr->image, bounding_rect, Scalar(0, 255, 0), 2);

        // 在黄色区域周围扩展区域进行灰色检测
        Rect surrounding_area(
            max(0, bounding_rect.x - surrounding_area_margin),
            max(0, bounding_rect.y - surrounding_area_margin),
            min(cv_ptr->image.cols, bounding_rect.width + 2 * surrounding_area_margin),
            min(cv_ptr->image.rows, bounding_rect.height + 2 * surrounding_area_margin));

        // 提取周围区域
        surrounding_area &= Rect(0, 0, cv_ptr->image.cols, cv_ptr->image.rows);
        Mat surrounding_roi = cv_ptr->image(surrounding_area);

        // 转换为HSV色彩空间
        Mat hsv_surrounding;
        cvtColor(surrounding_roi, hsv_surrounding, COLOR_BGR2HSV);

        // * 将旗帜标志位置为1
        RobotFlags.bFlagFound = true;

        // * 1. 识别灰色区域
        Mat gray_mask;
        inRange(hsv_surrounding, CvThreshold.lower_gray_, CvThreshold.upper_gray_, gray_mask);

        // 统计灰色像素点的数量
        int gray_pixel_count = countNonZero(gray_mask);

        // 如果灰色像素点数目超过阈值，说明附近有灰色区域
        if (gray_pixel_count > gray_pixel_threshold)
        {
            // * 找到战车营
            RobotFlags.btankFound = true;
            strFlag = "tankCamp";
#ifdef Desktop
            // 在图像上标记灰色区域
            for (int y = 0; y < gray_mask.rows; y++)
            {
                for (int x = 0; x < gray_mask.cols; x++)
                {
                    if (gray_mask.at<uchar>(y, x) > 0)
                    {
                        // 在图像上绘制灰色区域的像素点
                        circle(cv_ptr->image, Point(surrounding_area.x + x, surrounding_area.y + y), 1, Scalar(255, 0, 0), -1);
                    }
                }
            }
#endif

            // 输出信息，表示检测到灰色区域
            // RCLCPP_INFO(this->get_logger(), "Detected gray area near yellow block with %d gray pixels.", gray_pixel_count);
        }
        // * 2. 识别绿色区域
        Mat green_mask;
        Scalar green_lower = CvThreshold.lower_green_; // 绿色HSV下限
        Scalar green_upper = CvThreshold.upper_green_; // 绿色HSV上限
        inRange(hsv_surrounding, green_lower, green_upper, green_mask);
        int green_pixel_count = countNonZero(green_mask);
        int green_pixel_threshold = 300;
        // 如果绿色像素点数目超过阈值，说明附近有绿色区域
        if (green_pixel_count > green_pixel_threshold)
        {
            // * 找到步兵营
            RobotFlags.binfantryFound = true;
            strFlag = "infantryCamp";
// 在图像上标记绿色区域
#ifdef Desktop
            for (int y = 0; y < green_mask.rows; y++)
            {
                for (int x = 0; x < green_mask.cols; x++)
                {
                    if (green_mask.at<uchar>(y, x) > 0)
                    {
                        circle(cv_ptr->image, Point(surrounding_area.x + x, surrounding_area.y + y), 1, Scalar(0, 255, 0), -1);
                    }
                }
            }
#endif
        }

        // * 3. 识别蓝色区域
        Mat blue_mask;
        Scalar blue_lower = CvThreshold.lower_blue_; // 蓝色HSV下限
        Scalar blue_upper = CvThreshold.upper_blue_; // 蓝色HSV上限
        inRange(hsv_surrounding, blue_lower, blue_upper, blue_mask);
        int blue_pixel_count = countNonZero(blue_mask);
        int blue_pixel_threshold = 300;
        // 如果蓝色像素点数目超过阈值，说明附近有蓝色区域
        if (blue_pixel_count > blue_pixel_threshold)
        {
            // * 找到骑兵营
            RobotFlags.bcavalryFound = true;
            strFlag = "cavalryCamp";
// 在图像上标记蓝色区域
#ifdef Desktop
            for (int y = 0; y < blue_mask.rows; y++)
            {
                for (int x = 0; x < blue_mask.cols; x++)
                {
                    if (blue_mask.at<uchar>(y, x) > 0)
                    {
                        circle(cv_ptr->image, Point(surrounding_area.x + x, surrounding_area.y + y), 1, Scalar(0, 0, 255), -1);
                    }
                }
            }
#endif
        }
    }

    // 显示处理后的图像
    imshow("Detected Image", cv_ptr->image);
    cv::waitKey(1); // 每1ms刷新一次显示
#endif
    // 使用新函数检查像素数量是否超过20
    if (isAboveThreshold(image,20))
    {
        state_ = STATE_FIND_FLAG;
        auto voice = std_msgs::msg::String();
        voice.data = "found the flag";
        RCLCPP_INFO(this->get_logger(), "发布语音: %s", voice.data.c_str());
        voice_pub_->publish(voice);
    }
    state_ = STATE_FIND_END;
}


void MyRobotNodePid::oproadCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "CV Bridge Error: %s", e.what());
        return;
    }
    if (ImageState_ == IMAGE_STATE_FIND_BALL)
    {

        // * 将图像转换为 HSV 色彩空间
        cv::Mat imgOriginal = cv_ptr->image;
        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

        cv::Mat mask;
        cv::inRange(hsv_image, CvThreshold.Ball_lower, CvThreshold.Ball_upper, mask);

        int nTargetX = 0;
        int nTargetY = 0;
        int nPixCount = 0;
        int nImgWidth = mask.cols;
        int nImgHeight = mask.rows;
    }

    if (ImageState_ == IMAGE_STATE_FOLLOW_LINE)
    {
        // * 动态阈值二值化
        cv::Mat gray;
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
        // * 高斯滤波
        GaussianBlur(gray, gray, Size(5, 5), 0);
        // * 使用大津法进行二值化
        cv::Mat binary;
        cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

        // * 显示二值化结果
#ifdef Desktop
        cv::imshow("Binary Image", binary);
        cv::waitKey(1); // 这一步很重要，保持 OpenCV 窗口更新
#endif

        int rows = binary.rows;
        int cols = binary.cols;

        vector<int> leftBoundaries(rows, -1);
        vector<int> rightBoundaries(rows, -1);
        vector<int> middle(rows, 0);
        vector<int> auto_middle(rows, 0);
        float rows_area = 0.35;
        // * 第0行的中线从中间开始（假设为列数的一半）
        middle[rows] = cols / 2; // == 320
        auto_middle[rows] = cols / 2;
        for (int y = rows - 1; y >= int(rows * rows_area); --y)
        {
            int leftBoundary = -1, rightBoundary = -1;
            // * 使用上一行的中线位置来决定当前行的扫描起点
            int startX = middle[y + 1];
            // cout << "startX: " << startX << endl;
            // * 向左扫描，寻找左边界
            for (int z = startX; z > 2; --z)
            {
                uchar pixel = binary.at<uchar>(y, z);
                if ((pixel == 255 && binary.at<uchar>(y, z - 1) == 0 && binary.at<uchar>(y, z - 2) == 0) || z == 3)
                {
                    leftBoundary = z;
                    // cout << "leftBoundary: " << leftBoundary << endl;
                    break;
                }
            }
            // * 向右扫描，寻找右边界
            for (int x = startX; x < cols - 2; ++x)
            {
                uchar pixel = binary.at<uchar>(y, x);
                if ((pixel == 255 && binary.at<uchar>(y, x + 1) == 0 && binary.at<uchar>(y, x + 2) == 0) || x == cols - 3)
                {
                    rightBoundary = x;
                    // cout << "rightBoundary: " << rightBoundary << endl;
                    break;
                }
            }
            leftBoundaries[y] = leftBoundary;
            rightBoundaries[y] = rightBoundary;
            // * 记录当前行的中线，作为下一行的起点
            if (leftBoundary != -1 && rightBoundary != -1)
            {
                middle[y] = (leftBoundary + rightBoundary) / 2;
                // ! 此处auto_middle[y] 为摄像头相对图像中线的偏移
                // TODO 考虑修改为参数导入
                auto_middle[y] = middle[y] - 10;
            }
        }
    }
#ifdef Desktop
    // * 遍历所有行，画出边界和中线
    for (int y = rows; y > rows * rows_area; --y)
    {
        // 画左边界
        if (leftBoundaries[y] != -1)
        {
            circle(cv_ptr->image, Point(leftBoundaries[y], y), 2, Scalar(0, 255, 0), -1);
        }

        // 画右边界
        if (rightBoundaries[y] != -1)
        {
            circle(cv_ptr->image, Point(rightBoundaries[y], y), 2, Scalar(0, 255, 0), -1);
        }
        // 画中线
        if (middle[y] != 0)
        {
            circle(cv_ptr->image, Point(middle[y], y), 2, Scalar(0, 0, 255), -1);
        }
        cv::line(cv_ptr->image, cv::Point(cols / 2, 0), cv::Point(cols / 2, rows), cv::Scalar(255, 0, 0), 1);
        cv::line(cv_ptr->image, cv::Point(0, 300), cv::Point(cols, 300), cv::Scalar(255, 0, 0), 1);
    }

    imshow("Result Image", cv_ptr->image);
    // 等待用户按键，按任意键关闭窗口
    waitKey(1);
#endif
    // * 获取中线位置，并计算偏移量
    int middle_x = auto_middle[300];
    int middle_offset = middle_x - cols / 2;
    PIDControl.error = middle_offset;
    float speedX = 0.24;
    float speedZ = 0;
    float auto_kp = Update_kp_Speed(PIDControl.error, 20, 110);
    if (middle_offset > 0)
    {
        speedZ = PIDControl.error * auto_kp +
                 (PIDControl.error - PIDControl.last_error) * PIDControl.kd1;
#ifdef DEBUG
        cout << "speedZ: " << speedZ << endl;
#endif
        SetSpeed(speedX, -speedZ);
    }
    else if (middle_offset < 0)
    {
        speedZ = PIDControl.error * auto_kp +
                 (PIDControl.error - PIDControl.last_error) * PIDControl.kd1;
#ifdef DEBUG
        cout << "speedZ:" << speedZ << endl;
#endif
        SetSpeed(speedX, -speedZ);
    }
    else
    {
#ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "偏移为0");
#endif
        SetSpeed(speedX, 0);
    }
    PIDControl.last_error = PIDControl.error;
#ifdef FLAG
    cv::Mat hsv_flag;
    cvtColor(cv_ptr->image, hsv_flag, COLOR_BGR2HSV);

    // 显示处理后的图像
    imshow("Detected Image", cv_ptr->image);
    cv::waitKey(1); // 每1ms刷新一次显示
#endif
}
// 功能函数
void MyRobotNodePid::speak(std::string &voice)
{
    std_msgs::msg::String msg;
    msg.data = voice;
    voice_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "发布语音：%s", voice.c_str());
}
bool MyRobotNodePid::isAboveThreshold(cv::Mat &image, int threshold)
{
    // 计算非零像素的数量
    int nonZeroPixels = cv::countNonZero(image);
    // 判断非零像素数量是否超过阈值
    return nonZeroPixels > threshold;
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto pid = std::make_shared<MyRobotNodePid>();
    rclcpp::spin(pid);
    rclcpp::shutdown();
    return 0;
}
