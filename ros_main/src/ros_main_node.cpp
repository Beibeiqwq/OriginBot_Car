#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "cv_bridge/cv_bridge.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <vector>

//#define Desktop
#define DEBUG

using namespace std;
using namespace cv;

using geometry_msgs::msg::Twist;

#ifndef M_PI
#define M_PI 3.1415926535
#endif
/// @brief 程序状态
enum StructRobotState
{
    STATE_WAIT_CMD,
    STATE_WAIT_ENTER,
    STATE_STRAIGHT,
    STATE_ROTATE,
    STATE_ARRIVE_GRANRAY,
    STATE_FIND_BALL
};
/// @brief 图像回调状态
enum StructImageState
{
    IMAGE_STATE_WAIT,
    IMAGE_STATE_FIND_FLAG,
    IMAGE_STATE_FOLLOW_LINE,
    IMAGE_STATE_FIND_BALL
};

class RosMainNode : public rclcpp::Node
{
public:
    RosMainNode() : Node("ros_main_node"), state_(STATE_WAIT_CMD)
    {
        start_detect_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/start_detect", 10,
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                // * 订阅红绿灯识别节点
                startDetectCallback(msg);
            });

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg)
            {
                // * 图像回调
                imageCallback(msg);
            });

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg)
            {
                // * IMU回调
                // ! 已废弃 IMU有漂移
                imuCallback(msg);
            });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]()
            {
                // * 状态机
                timerCallback();
            });


        robot_voice_pub_ = this->create_publisher<std_msgs::msg::String>("/robot_voice", 10);
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // * 三个兵营的到达标志位
        CampFlags.cavalryCampArrived = false;
        CampFlags.infantryCampArrived = false;
        CampFlags.tankCampArrived = false;

        // * 机器运动控制节点
        // TODO 考虑修改为参数导入
        RobotRun.bTurnLeft = false;
        RobotRun.bTurnRight = false;
        RobotRun.nTurnOffset = 0;
        RobotRun.currentYaw = 0;
        RobotRun.targetYaw = 0;
        RobotRun.fMaxSpeed = 0.3;
        RobotRun.fMaxTurn = 1.3;
        
        // * 机器识别标志位
        RobotFlags.bBallFound = false;
        RobotFlags.bFlagFound = false;
        RobotFlags.bGranaryFound = false;
        
        // * 机器识别计数
        RobotCount.nFlagCount = 0;
        RobotCount.nStartCount = 0;

        // * 红球颜色阈值
        // TODO 考虑修改变量名 改为参数导入？
        CvThreshold.lower_red_ = cv::Scalar(0, 180, 200);
        CvThreshold.upper_red_ = cv::Scalar(179, 255, 255);

        // * PID参数
        // TODO 考虑修正 改为参数导入
        PIDControl.kp1 = 0.011;
        PIDControl.ki1 = 0.0;
        PIDControl.kd1 = 0.003;
        PIDControl.kp2 = 0.002;
        PIDControl.ki2 = 0.0;
        PIDControl.kd2 = 0.002;
        PIDControl.error = 0;
        PIDControl.last_error = 0;

        // * 启动程序
        int _check_flag;
        cout << "[Init]键入任意数字开始.... 按CTRL+Z退出" << endl;
        cin >> _check_flag;
        ImageState_ = IMAGE_STATE_FOLLOW_LINE;
        state_ = STATE_WAIT_ENTER;
    }

private:
    void timerCallback()
    {
        switch (state_)
        {
        case STATE_WAIT_CMD:
            // * 初始化完成
            break;

        case STATE_WAIT_ENTER:
            // * 等待红绿灯
            break;
        case STATE_STRAIGHT:
            // * 识别旗帜赛道
            // ! 此处应调整为直道PID 等待识别完成
            // TODO 识别到旗帜后，进入调整为弯道PID
            if (RobotCount.nFlagCount == 3)
            {
                state_ = STATE_ROTATE;
                RCLCPP_INFO(this->get_logger(), "State changed to STATE_ROTATE.");
                break;
            }
            // 此处SetSpeed由图像回调控制 等待修复
            SetSpeed(0.24, 0);
            if (RobotFlags.bFlagFound)
            {
                SetSpeed(0.24, 0);
                ProcessCamp(strFlag);
            }

            break;
        case STATE_ROTATE:
        // * 弯道避障赛道
        // ! 此处应调整为弯道PID 等待过弯
        // * 识别到粮仓旗帜后 进入找球行为
            SetSpeed(0.4,0); //此处Speed由图像回调控制 等待修复
            if(RobotRun.bTurnLeft == true)
            {
                SetSpeed(0,0.4);
            }
            else if(RobotRun.bTurnRight == true)
            {
                SetSpeed(0,-0.4);
            }
            else
            {
                SetSpeed(0.4,0);
            }
            // * 找到粮仓后 状态切换
            if(RobotFlags.bGranaryFound == true)
            {
                state_ = STATE_ARRIVE_GRANRAY;
                break; 
            }

            break;
        case STATE_ARRIVE_GRANRAY:
            // * 播报后停止四秒 寻找球
            // ! 回调函数也得停止 
            // TODO 等待添加回调函数位
            Speak("到达粮仓");
            std::this_thread::sleep_for(std::chrono::seconds(4));
            state_ = STATE_FIND_BALL;
            break;
        case STATE_FIND_BALL:
            if(!RobotFlags.bBallFound)
            {
                FindBall(); //未编写
            }
            break;
        default:
            break;
        }
    }

    struct StructCampFlags
    {
        bool cavalryCampArrived  = false;
        bool infantryCampArrived = false;
        bool tankCampArrived     = false;
    };
    struct StructRobotRun
    {
        bool bTurnLeft  = false;
        bool bTurnRight = false;

        double nTurnOffset = 0;
        double currentYaw = 0;
        double targetYaw = 0;
        double targetPoint = 0;

        float fMaxSpeed = 0.4;
        float fMaxTurn  = 0.5;
    };
    struct StructRobotFlag
    {
        bool bBallFound = false;
        bool bFlagFound = false;
        bool bGranaryFound = false;
    };
    struct StructRobotCount
    {
        int nFlagCount  = 0;
        int nStartCount = 0;
    };
    struct StructCvThreshold
    {
        cv::Scalar lower_red_ = cv::Scalar(0, 180, 200);   // 最小的红色
        cv::Scalar upper_red_ = cv::Scalar(179, 255, 255); // 最大的红色
    };
    struct StructPIDControl
    {
        float kp1;
        float ki1;
        float kd1;

        float kp2;
        float ki2;
        float kd2;

        float error;
        float last_error;

    };

    std::string strFlag;
    // * 结构体初始化
    StructCampFlags CampFlags;
    StructRobotRun RobotRun;
    StructRobotFlag RobotFlags;
    StructRobotCount RobotCount;
    StructRobotState state_;
    StructImageState ImageState_;
    StructCvThreshold CvThreshold;
    StructPIDControl PIDControl;

    // * 订阅器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_detect_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // * 发布器
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_voice_pub_;
    

    // * 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // * 回调函数
    void startDetectCallback(const std_msgs::msg::String::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    // * 功能函数
    void Speak(std::string strToSpeak);
    void SetSpeed(float lineX,float angZ);
    void RobotCtl(const Twist& msg) const;
    void FindBall();
    // * 程序功能函数
    void ProcessCamp(const std::string& strFlag);
    void CalcPID();
    float Update_kp_Speed(int error,int absmin, int absmax);

};
/**********************************************************/
/*                       回调函数                          */
/**********************************************************/
void RosMainNode::startDetectCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if (msg->data == "open")
    {
        RobotCount.nStartCount++;
        if (RobotCount.nStartCount > 20)
        {
            state_ = STATE_WAIT_CMD;
            RCLCPP_INFO(this->get_logger(), "State changed to STATE_WAIT_CMD.");
        }
    }
    else
    {
        RobotCount.nStartCount = 0;
    }
}

void RosMainNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    const auto &orientation = msg->orientation;

    tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    //RCLCPP_INFO(this->get_logger(), "Yaw: %f radians", yaw);

    RobotRun.currentYaw = yaw * 180.0 / M_PI;
    //RCLCPP_INFO(this->get_logger(), "Yaw: %f degrees", RobotRun.currentYaw);
}

void RosMainNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
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

        // 将图像转换为 HSV 色彩空间
        cv::Mat imgOriginal = cv_ptr->image;
        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

        // vector<cv::Mat> hsvSplit;
        // split(hsv_image, hsvSplit);
        // equalizeHist(hsvSplit[2], hsvSplit[2]);
        // merge(hsvSplit, hsv_image);

        // 阈值操作，获取红色区域
        cv::Mat mask;
        cv::inRange(hsv_image, CvThreshold.lower_red_, CvThreshold.upper_red_, mask);

        // cv::Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
        // morphologyEx(mask, mask, MORPH_OPEN, element);
        // morphologyEx(mask, mask, MORPH_CLOSE, element);

        int nTargetX = 0;
        int nTargetY = 0;
        int nPixCount = 0;
        int nImgWidth = mask.cols;
        int nImgHeight = mask.rows;
        for (int y = 0; y < nImgHeight; y++)
        {
            for (int x = 0; x < nImgWidth; x++)
            {
                if (mask.data[y * nImgWidth + x] == 255)
                {
                    nTargetX += x;
                    nTargetY += y;
                    nPixCount++;
                }
            }
        }
        geometry_msgs::msg::Twist vel_cmd;
        float fVelFoward = 0;
        float fVelTurn = 0;
        if (nPixCount > 200)
        {
            nTargetX /= nPixCount;
            nTargetY /= nPixCount;
            printf("Target (%d, %d) PixelCount = %d\n", nTargetX, nTargetY, nPixCount);
#ifdef Desktop
            Point line_begin = Point(nTargetX - 10, nTargetY);
            Point line_end = Point(nTargetX + 10, nTargetY);
            line(imgOriginal, line_begin, line_end, Scalar(255, 0, 0), 3);
            line_begin.x = nTargetX;
            line_begin.y = nTargetY - 10;
            line_end.x = nTargetX;
            line_end.y = nTargetY + 10;
            line(imgOriginal, line_begin, line_end, Scalar(255, 0, 0), 3);
#endif
            fVelFoward = (nImgHeight / 2 - nTargetY) * 0.002;
            fVelTurn = (nImgWidth / 2 - nTargetX) * 0.003;
#ifdef DEBUG
            printf("fVelFoward:%f  nImgHeight / 2 = %d", fVelFoward, nImgHeight / 2);
            printf("fVelTurn:%f   nImgWidth /2 = %d", fVelTurn, nImgWidth / 2);
#endif
            // geometry_msgs::msg::Twist vel_cmd;

            // vel_cmd.linear.x = fVelFoward;
            // vel_cmd.angular.z = fVelTurn;
            // ! 丑陋的滤波
            // TODO 考虑更改
            if (fVelFoward >= 0 && fVelFoward <= 0.04)
            {
                fVelFoward = 0;
            }
            else if (fVelFoward <= 0 && fVelFoward >= -0.04)
            {
                fVelFoward = 0;
            }
            if (fVelTurn >= 0 && fVelTurn <= 0.04)
            {
                fVelTurn = 0;
            }
            else if (fVelTurn <= 0 && fVelTurn >= -0.04)
            {
                fVelTurn = 0;
            }
            SetSpeed(fVelFoward, fVelTurn);
        }
        else
        {
            // FindBall(); 记录下上一次的转向速度 寻找小球的方向
            RobotFlags.bBallFound = false;
#ifdef DEBUG
            printf("Target disappeared...\n");
#endif
            // vel_cmd.linear.x = 0;
            // vel_cmd.linear.y = 0;
            // vel_cmd.linear.z = 0;
            // vel_cmd.angular.x = 0;
            // vel_cmd.angular.y = 0;
            // vel_cmd.angular.z = 0;
            fVelFoward = 0;
            fVelTurn = 0;
        }
        SetSpeed(fVelFoward, fVelTurn);
        // twist_publisher_->publish(vel_cmd);
#ifdef Desktop
        imshow("Result", mask);
        imshow("RGB", imgOriginal);
        cv::waitKey(5);
#endif
        // // 查找轮廓
        // std::vector<std::vector<cv::Point>> contours;
        // cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        // // 如果找到红色小球的轮廓
        // if (!contours.empty())
        // {
        //     // 选取最大轮廓（假设小球为最大目标）
        //     auto max_contour = *std::max_element(contours.begin(), contours.end(),
        //                                         [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
        //                                         {
        //                                             return cv::contourArea(a) < cv::contourArea(b);
        //                                         });

        //     // 计算小球的质心
        //     cv::Moments moments = cv::moments(max_contour);
        //     int cx = static_cast<int>(moments.m10 / moments.m00);
        //     int cy = static_cast<int>(moments.m01 / moments.m00);

        //     // 绘制轮廓和质心
        //     cv::drawContours(cv_ptr->image, contours, -1, cv::Scalar(0, 255, 0), 2);
        //     cv::circle(cv_ptr->image, cv::Point(cx, cy), 5, cv::Scalar(0, 0, 255), -1);

        //     // 计算小球相对于图像中心的偏移
        //     int x_offset = cx - cv_ptr->image.cols / 2;
        //     int y_offset = cy - cv_ptr->image.rows / 2;

        //     // 控制小车运动，根据偏移调整速度
        //     geometry_msgs::msg::Twist cmd;
        //     cmd.linear.x = 0.0;                // 前进速度
        //     cmd.angular.z = -x_offset * 0.005; // 根据x偏移调整转向
        //     twist_publisher_->publish(cmd);
        // }

        // // 显示结果图像
        // cv::imshow("Ball Tracking", cv_ptr->image);
        // cv::waitKey(1);
    }

    if (ImageState_ == IMAGE_STATE_FOLLOW_LINE)
    {
        // 动态阈值二值化
        cv::Mat gray;
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
        // 高斯滤波
        GaussianBlur(gray, gray, Size(5, 5), 0);
        // 使用大津法进行二值化
        cv::Mat binary;
        cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

        // 显示二值化结果
#ifdef Desktop
        cv::imshow("Binary Image", binary);
        cv::waitKey(1); // 这一步很重要，保持 OpenCV 窗口更新
#endif

        int rows = binary.rows;
        int cols = binary.cols;

        // int pixelArray[rows][cols];

        // // 遍历图像的每个像素
        // for (int y = 0; y < rows; ++y)
        // {
        //     for (int x = 0; x < cols; ++x)
        //     {
        //         // 获取当前像素的值
        //         uchar pixelValue = binary.at<uchar>(y, x);

        //         // 将像素值存储到二维数组中
        //         pixelArray[y][x] = pixelValue; // 0 - 黑色, 255 - 白色
        //     }
        // }

        // 存储每一行的中线位置
        vector<int> leftBoundaries(rows, -1);
        vector<int> rightBoundaries(rows, -1);
        vector<int> middle(rows, 0);
        vector<int> auto_middle(rows, 0);
        float rows_area = 0.35;
        // 第0行的中线从中间开始（假设为列数的一半）
        middle[rows] = cols / 2; // == 320
        auto_middle[rows] = cols / 2;
        for (int y = rows-1; y >= int(rows*rows_area); --y)
        {
            int leftBoundary = -1, rightBoundary = -1;
            // 使用上一行的中线位置来决定当前行的扫描起点
            int startX = middle[y+1];
            // cout << "startX: " << startX << endl;
            // 向左扫描，寻找左边界
            for (int z = startX; z > 2; --z)
            {
                uchar pixel = binary.at<uchar>(y, z);
                if ((pixel == 255 && binary.at<uchar>(y, z - 1) == 0 && binary.at<uchar>(y, z - 2) == 0)
                || z == 3)
                {
                    leftBoundary = z;
                    //cout << "leftBoundary: " << leftBoundary << endl;
                    break;
                }
            }

            // 向右扫描，寻找右边界
            for (int x = startX; x < cols -2 ; ++x)
            {
                uchar pixel = binary.at<uchar>(y, x);
                if ((pixel == 255 && binary.at<uchar>(y, x + 1) == 0 && binary.at<uchar>(y, x + 2) == 0)
                || x == cols - 3)
                {
                    rightBoundary = x;
                    //cout << "rightBoundary: " << rightBoundary << endl;
                    break;
                }
            }
            leftBoundaries[y] = leftBoundary;
            rightBoundaries[y] = rightBoundary;
            // 记录当前行的中线，作为下一行的起点
            if (leftBoundary != -1 && rightBoundary != -1)
            {
                middle[y] = (leftBoundary + rightBoundary) / 2;
                auto_middle[y] = middle[y] - 10;
            }

            // if (leftBoundary != -1)
            // {
            //     circle(cv_ptr->image, Point(leftBoundary, y), 1, Scalar(0, 255, 0), -1); // 半径为1的绿色像素点
            // }
            // if (rightBoundary != -1)
            // {
            //     circle(cv_ptr->image, Point(rightBoundary, y), 1, Scalar(0, 255, 0), -1); // 半径为1的绿色像素点
            // }

            // // 绘制中线的像素点（红色）
            // if (middle[y] != 0)
            // {
            //     circle(cv_ptr->image, Point(middle[y], y), 1, Scalar(0, 0, 255), -1); // 半径为1的红色像素点
            // }
            // imshow("Result", cv_ptr->image);
            // waitKey(1);
        }
#ifdef Desktop
        // 遍历所有行，画出边界和中线
        for (int y = rows; y > rows*rows_area ; --y)
        {
            // 画左边界圆
            if (leftBoundaries[y] != -1)
            {
                circle(cv_ptr->image, Point(leftBoundaries[y], y), 2, Scalar(0, 255, 0), -1); 
            }

            // 画右边界圆
            if (rightBoundaries[y] != -1)
            {
                circle(cv_ptr->image, Point(rightBoundaries[y], y), 2, Scalar(0, 255, 0), -1); 
            }

            // 画中线圆
            if (middle[y] != 0)
            {
                circle(cv_ptr->image, Point(middle[y], y), 2, Scalar(0, 0, 255), -1); 
            }
            cv::line(cv_ptr->image, cv::Point(cols/2, 0), cv::Point(cols/2, rows), cv::Scalar(255, 0, 0), 1);
            cv::line(cv_ptr->image, cv::Point(0, 300), cv::Point(cols, 300), cv::Scalar(255, 0, 0), 1);
        }

        imshow("Result Image", cv_ptr->image);

        // 等待用户按键，按任意键关闭窗口
        waitKey(1);
#endif

        // 获取中线位置，并计算偏移量
        int middle_x = auto_middle[300];
        int middle_offset = middle_x - cols / 2;
        PIDControl.error = middle_offset;
        float speedX = 0.24;
        float speedZ = 0;
        float auto_kp = Update_kp_Speed(PIDControl.error,20,110);
        //float auto_kp = PIDControl.kp1;
        RCLCPP_INFO(this->get_logger(),"Middle Offset: %d", middle_offset);
        if (middle_offset > 0)
        {
            // 偏移向左，小车向右转
            speedZ = PIDControl.error * auto_kp +
                     (PIDControl.error - PIDControl.last_error) * PIDControl.kd1;
#ifdef DEBUG
            //RCLCPP_INFO(this->get_logger(), "偏移向左，小车向右转");
            cout << "speedZ: " << speedZ << endl;
#endif
            SetSpeed(speedX, -speedZ);
        }
        else if (middle_offset < 0)
        {
            // 偏移向右，小车向左转
            speedZ = PIDControl.error * auto_kp +
                     (PIDControl.error - PIDControl.last_error) * PIDControl.kd1;
#ifdef DEBUG
            //RCLCPP_INFO(this->get_logger(), "偏移向右，小车向左转");
            cout << "speedZ:" << speedZ << endl;
#endif
            SetSpeed(speedX, -speedZ);
        }
        else
        {
            // 偏移为0，小车停止
#ifdef DEBUG
            RCLCPP_INFO(this->get_logger(), "偏移为0");
#endif
            SetSpeed(speedX, 0);
        }
        PIDControl.last_error = PIDControl.error;
    }
}
/**********************************************************/
/*                     机器功能函数                         */
/**********************************************************/
void RosMainNode::Speak(std::string strToSpeak)
{
    std_msgs::msg::String voice_msg;
    voice_msg.data = strToSpeak;
    robot_voice_pub_->publish(voice_msg);
}

/// @brief 速度控制函数
/// @param lineX 
/// @param angZ 
void RosMainNode::SetSpeed(float lineX,float angZ)
{
    // ! angZ > 0 左转
    // ! angZ < 0 右转
    //auto twist = geometry_msgs::msg::Twist();
    geometry_msgs::msg::Twist twist;
    if(lineX >= RobotRun.fMaxSpeed)
        lineX = RobotRun.fMaxSpeed;
    else if(lineX <= -RobotRun.fMaxSpeed)
        lineX = -RobotRun.fMaxSpeed;
    
    if(angZ >= RobotRun.fMaxTurn)
        angZ = RobotRun.fMaxTurn;
    else if(angZ <= -RobotRun.fMaxTurn)
        angZ = -RobotRun.fMaxTurn;
    //auto twist = std::make_shared<Twist>();
    twist.linear.x = lineX;
    twist.angular.z = angZ;
    twist_publisher_->publish(twist);
}

void RosMainNode::RobotCtl(const Twist &msg) const
{
    // ! 使用时需传入指针  RobotCtl(*pose);
    twist_publisher_->publish(msg);
}

/// @brief 找球行为
void RosMainNode::FindBall()
{
    auto message = geometry_msgs::msg::Twist();

    if (RobotFlags.bBallFound == true)
    {
        message.angular.z = -1.0 * (RobotRun.targetPoint - 480) / 300.0;
        message.linear.x = 0.24;
    }
    else
    {
        message.angular.z = 0.15;
        message.linear.x = 0;
    }
    message.linear.y = 0.0;
    message.linear.z = 0.0;
    message.angular.x = 0.0;
    message.angular.y = 0.0;
    twist_publisher_->publish(message);
}
/**********************************************************/
/*                     程序功能函数                         */
/**********************************************************/

/// @brief 旗帜处理函数 -- 语音
/// @param strFlag 
void RosMainNode::ProcessCamp(const std::string& strFlag) {
    std::map<std::string, std::string> campMessages = {
        {"cavalryCamp", "到达骑兵营"},
        {"infantryCamp", "到达步兵营"},
        {"tankCamp", "到达战车营"}};

    auto it = campMessages.find(strFlag);
    if (it != campMessages.end())
    {
        if (strFlag == "cavalryCamp" && !CampFlags.cavalryCampArrived)
        {
            CampFlags.cavalryCampArrived = true;
            RobotCount.nFlagCount++;
            Speak(it->second);
        }
        else if (strFlag == "infantryCamp" && !CampFlags.infantryCampArrived)
        {
            CampFlags.infantryCampArrived = true;
            RobotCount.nFlagCount++;
            Speak(it->second);
        }
        else if (strFlag == "tankCamp" && !CampFlags.tankCampArrived)
        {
            CampFlags.tankCampArrived = true;
            RobotCount.nFlagCount++;
            Speak(it->second);
        }
    }
}

void RosMainNode::CalcPID()
{

}
/// @brief PID-Kp参数更新
/// @param error 
/// @param absmin 
/// @param absmax 
/// @return Kp参数
// ! 未进行弯道和直道判断 考虑最小二乘法
// TODO 优化该函数
float RosMainNode::Update_kp_Speed(int error,int absmin, int absmax)
{
    if (abs(error) < absmin)
        return PIDControl.kp1;// 直道
    else if (abs(error) > absmax)
        return PIDControl.kp1;// 弯道
    else
        return PIDControl.kp1;//默认
}
/**********************************************************/
/*                       主函数                            */
/**********************************************************/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RosMainNode>());
    rclcpp::shutdown();
    return 0;
}
