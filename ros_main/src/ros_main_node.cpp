#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <vector>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "cv_bridge/cv_bridge.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#define IMAGE_H 480
#define IMAGE_W 640

//加权控制
const int Weight[160] =
    {
        15, 15, 13, 13, 13, 13, 11, 11, 11, 11,
        9, 9, 9, 9, 7, 7, 7, 5, 3, 1, // 图像最远端00 ——09 行权重
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        // 图像最远端10 ——19 行权重
        1, 1, 1, 1, 1, 1, 1, 3, 4, 5,          // 图像最远端20 ——29 行权重
        //1, 1, 1, 1, 1, 1, 1, 1, 1, 1,          // 图像最远端30 ——39 行权重
        //1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
        1, 3, 5, 7, 9, 11, 13, 13, 15, 15, 
        5, 15, 15, 17, 17, 17, 17, 19, 20, 20,
        15, 15, 15, 17, 17, 17, 17, 19, 20, 20,    
        15, 15, 15, 17, 17, 17, 17, 19, 20, 20 // 图像最远端60 ——69 行权重
};

//#define Desktop

#ifndef Desktop
#include "ai_msgs/msg/perception_targets.hpp"
#endif


#define DEBUG
//#define YOLO_DEBUG
//#define FLAG // ! 效果过差 改用Yolo
using namespace std;
using namespace cv;

using geometry_msgs::msg::Twist;

#ifndef M_PI
#define M_PI 3.1415926535
#endif

/// @brief 程序状态
enum StructRobotState
{
    STATE_WAIT_CMD,       // * 等待命令
    STATE_WAIT_ENTER,     // * 等待红绿灯
    STATE_STRAIGHT,       // * 直道赛道 -- 识别旗帜
    STATE_ROTATE,         // * 弯道赛道 -- 识别障碍物 + 粮仓
    STATE_ARRIVE_GRANRAY, // * 粮仓  -- 等待3s
    STATE_FIND_BALL       // * 找红球 -- HSV
};
/// @brief 图像回调状态
enum StructImageState
{
    IMAGE_STATE_WAIT,       // * 初始化
    IMAGE_STATE_FOLLOW_LINE,// * 弯道赛道 || 障碍物
    IMAGE_STATE_FIND_BALL   // * 寻找红球
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
#ifdef Desktop
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg)
            {
                // * 图像回调
                imageCallback(msg);
            });
#endif
#ifndef Desktop
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera1_ns/image_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg)
            {
                // * 图像回调
                imageCallback(msg);
            });
#endif
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
#ifndef Desktop
        yolo_sub_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
            "hobot_dnn_detection", 10,
            [this](const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
            {
                // * Yolo回调
                yoloCallback(msg);
            });
#endif
        // * 语音播放
        robot_voice_pub_ = this->create_publisher<std_msgs::msg::String>("/robotvoice", 10);
        // * 速度发布
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // * 三个兵营的到达标志位
        CampFlags.cavalryCampArrived = false;
        CampFlags.infantryCampArrived = false;
        CampFlags.tankCampArrived = false;

        // * 机器运动控制节点
        // TODO 考虑修改为参数导入
        // ! 惯导方案 废弃
        // RobotRun.bTurnLeft = false;
        // RobotRun.bTurnRight = false;
        // RobotRun.nTurnOffset = 0;
        // RobotRun.currentYaw = 0;
        // RobotRun.targetYaw = 0;

        RobotRun.fMaxSpeed = 0.3; // * 小车最大速度
        RobotRun.fMaxTurn = 1.3;  // * 小车最大转向速度

        // * 机器识别标志位
        RobotFlags.bBallFound = false; // * 找到红球 -- HSV
        RobotFlags.bFlagFound = false; // * 找到旗帜 -- YOLOV5
        RobotFlags.bBallStable = false;
        // ! 废弃 改用yolo
        // RobotFlags.bgranaryFound = false;
        // RobotFlags.bcavalryFound = false;
        // RobotFlags.bgranaryFound = false;
        // RobotFlags.btankFound = false;
        // * 机器识别计数
        RobotCount.nFlagCount = 0; // * 旗帜计数 -- 语音识别函数中
        RobotCount.nStartCount = 0; // * 红绿灯计数 -- 红绿灯回调

        CvThreshold.Ball_lower = cv::Scalar(0,110,255);
        CvThreshold.Ball_upper = cv::Scalar(255,255,255);

        // * PID参数
        // TODO 考虑修正 改为参数导入
        PIDControl.kp = 0.005;
        PIDControl.ki = 0.0;
        PIDControl.kd = 0.0;
        // * 弯道PID
        PIDControl.kp1 = 0.03;
        PIDControl.ki1 = 0.0;
        PIDControl.kd1 = 0.003;
        // * 直道PID
        PIDControl.kp2 = 0.002;
        PIDControl.ki2 = 0.0;
        PIDControl.kd2 = 0.002;

        PIDControl.error = 0;
        PIDControl.last_error = 0;

        // * 启动程序
        int _state_flag;
        cout << "[State]请输入当前程序状态:" << endl;
        cout << "1: 等待红绿灯 2: 直线 3: 弯道 4: 找球" << endl;
        cin >> _state_flag;
        int _image_flag;
        cout << "[State]请输入当前图像状态:" << endl;
        cout << "1: 巡线 2: 找球 " << endl;
        cin >> _image_flag;
        ImageState_ = Update_image_state(_image_flag);
        state_ = Update_state(_state_flag);
        cout <<"[State]当前状态为:" << state_ << endl;
        cout <<"[State]当前图像状态为:" << ImageState_ << endl;
        start_time = this->now();
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
            ImageState_ = IMAGE_STATE_WAIT;
            current_time = this->now();
            duration = current_time - start_time;
            if(duration.seconds() > 20)
            {
                state_ = STATE_STRAIGHT;
                RCLCPP_INFO(this->get_logger(), "State changed to STATE_STRAIGHT.");
                break;
            }
            break;
        case STATE_STRAIGHT:
            // * 识别旗帜赛道
            // ! 此处应调整为直道PID 等待识别完成
            // TODO 识别到旗帜后，进入调整为弯道PID

            // * 1.
            cout << "[State] 直道赛道" << endl;
            ImageState_ = IMAGE_STATE_WAIT;
            SetSpeed(0.2,0);

            // * 2.
            if (RobotCount.nFlagCount == 3)
            {
                state_ = STATE_ROTATE;
                RCLCPP_INFO(this->get_logger(), "State changed to STATE_ROTATE.");
                break;
            }

            // * 3.
            if (RobotFlags.bFlagFound)
            {
                //SetSpeed(0.24, 0);
                ProcessCamp(strFlag);
            }

            break;
        case STATE_ROTATE:
            // * 弯道避障赛道
            // ! 此处应调整为弯道PID 等待过弯
            // * 识别到粮仓旗帜后 进入找球行为

            // * 1.
            ImageState_ = IMAGE_STATE_FOLLOW_LINE;
            cout << "[State] 弯道避障赛道" << endl;
            cout << "[State] 当前图像状态为:" << ImageState_ << endl;
            if (RobotFlags.bFlagFound)
            {
                //SetSpeed(0.24, 0);
                ProcessCamp(strFlag);
            }
            // * 找到粮仓后 状态切换
            // * 2.
            if (RobotFlags.bgranaryFound == true)
            {
                state_ = STATE_ARRIVE_GRANRAY;
                break;
            }

            break;
        case STATE_ARRIVE_GRANRAY:
            // * 播报后停止四秒 寻找球
            // ! 回调函数也得停止
            // TODO 等待添加回调函数位
            // * 1. Reset
            SetSpeed(0, 0);
            state_ = STATE_WAIT_CMD;
            ImageState_ = IMAGE_STATE_WAIT;
            std::this_thread::sleep_for(std::chrono::seconds(3));

            // * 2. Set
            state_ = STATE_FIND_BALL;
            break;
        case STATE_FIND_BALL:
            // TODO 寻球逻辑 等待编写
            ImageState_ = IMAGE_STATE_FIND_BALL;
            if (!RobotFlags.bBallFound)
            {
                //FindBall(); // 未编写
                ImageState_ = IMAGE_STATE_FIND_BALL;
                //SetSpeed(0.1,0);
            }
            if(RobotFlags.bBallStable == true)
            {
                //ImageState_ = IMAGE_STATE_FOLLOW_LINE;
                //SetSpeed(0.2, 0);
                PushBall();
                cout << "Ball Found!" << endl;
            }
            break;
        default:
            break;
        }
    }

    struct StructCampFlags
    {
        bool cavalryCampArrived = false;  // *骑兵营
        bool infantryCampArrived = false; // *步兵营
        bool tankCampArrived = false;     // *战车营
        bool granaryArrived = false;      // *粮仓
    };
    struct StructRobotRun
    {
        bool bTurnLeft = false;
        bool bTurnRight = false;

        double nTurnOffset = 0;
        double currentYaw = 0;
        double targetYaw = 0;
        double targetPoint = 0;

        float fMaxSpeed = 0.4;
        float fMaxTurn = 0.5;
    };
    struct StructRobotFlag
    {
        bool bBallFound = false;
        bool bFlagFound = false;
        bool bBallStable = false;
        bool bgranaryFound = false;  // *粮仓
        bool bcavalryFound = false;  // *骑兵营
        bool binfantryFound = false; // *步兵营
        bool btankFound = false;     // *战车营
    };
    struct StructRobotCount
    {
        int nFlagCount = 0;
        int nStartCount = 0;
    };
    struct StructCvThreshold
    {
#ifdef FLAG
        cv::Scalar lower_red_ = cv::Scalar(0, 180, 200);   // 最小的红色
        cv::Scalar upper_red_ = cv::Scalar(179, 255, 255); // 最大的红色

        cv::Scalar lower_yellow_ = cv::Scalar(13, 255, 65);  // 黄色的下界
        cv::Scalar upper_yellow_ = cv::Scalar(40, 255, 255); // 黄色的上界

        cv::Scalar lower_gray_ = cv::Scalar(0, 0, 110);     // 灰色的下界
        cv::Scalar upper_gray_ = cv::Scalar(179, 110, 148); // 灰色的上界

        cv::Scalar lower_blue_ = cv::Scalar(100, 220, 55);
        cv::Scalar upper_blue_ = cv::Scalar(124, 255, 255);

        cv::Scalar lower_green_ = cv::Scalar(53, 163, 30);
        cv::Scalar upper_green_ = cv::Scalar(78, 255, 255);
#endif
        cv::Scalar Ball_lower = cv::Scalar(0,110,255);
        cv::Scalar Ball_upper = cv::Scalar(179,255,255);
    };
    struct StructPIDControl
    {
        float kp;
        float ki;
        float kd;

        float kp1;
        float ki1;
        float kd1;

        float kp2;
        float ki2;
        float kd2;

        float error;
        float last_error;
    };

    std::string strFlag = "";
    int nVelForwardStable = 0;
    int nVelTurnStable = 0;
    float fVelTurnCount = 0;
    // * 结构体初始化
    StructCampFlags CampFlags;
    StructRobotRun RobotRun;
    StructRobotFlag RobotFlags;
    StructRobotCount RobotCount;
    int state_;
    int ImageState_;
    StructCvThreshold CvThreshold;
    StructPIDControl PIDControl;

    // * 订阅器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr start_detect_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
#ifndef Desktop
    rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr yolo_sub_;
#endif

    // * 发布器
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_voice_pub_;

    // * 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time;
    rclcpp::Time current_time;
    rclcpp::Duration duration = current_time - start_time;

    // * 回调函数
    void startDetectCallback(const std_msgs::msg::String::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
#ifndef Desktop
    void yoloCallback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);
#endif

    // * 功能函数
    void Speak(std::string strToSpeak);
    void SetSpeed(float lineX, float angZ);
    void RobotCtl(const Twist &msg) const;
    void FindBall();
    void PushBall();
    // * 程序功能函数
    void ProcessCamp(const std::string &strFlag);
    void PID_Choose(int cmd);
    float Update_kp_Speed(int error, int absmin, int absmax);
    int Update_state(int flag);
    int Update_image_state(int flag);
};
/**********************************************************/
/*                       回调函数                          */
/**********************************************************/
void RosMainNode::PushBall()
{
    if(fVelTurnCount > 0)
    {
        SetSpeed(0.2,-0.3);
    }
    if(fVelTurnCount < 0)
    {
        SetSpeed(0.2,0.3);
    }
    if(fVelTurnCount == 0)
    {
        SetSpeed(0.2,0);
    }
}
int RosMainNode::Update_state(int flag)
{
    switch (flag)
    {
    case 1:
        return STATE_WAIT_ENTER;
    case 2:
        return STATE_STRAIGHT;
    case 3: 
        return STATE_ROTATE;
    case 4:
        return STATE_FIND_BALL;
    }
    return 1;
}

int RosMainNode::Update_image_state(int flag)
{
    switch (flag)
    {
    case 1:
        return IMAGE_STATE_FOLLOW_LINE;
    case 2:
        return IMAGE_STATE_FIND_BALL;
    }
    return 1;
}
void RosMainNode::startDetectCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if(state_ == STATE_WAIT_ENTER)
    {
        if (msg->data == "open")
        {
            RobotCount.nStartCount++;
            if (RobotCount.nStartCount > 20)
            {
                state_ = STATE_STRAIGHT;
                RCLCPP_INFO(this->get_logger(), "State changed to STATE_STRAIGHT.");
            }
        }
        else
        {
            RobotCount.nStartCount = 0;
        }

    }
}
#ifndef Desktop
void RosMainNode::yoloCallback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
{
    // 遍历所有目标对象并输出相关信息
    for (size_t num = 0; num < msg->targets.size(); ++num)
    {
        const auto &target = msg->targets[num];
        const auto &roi = target.rois[0].rect;
        // * 目标面积 -- 判断是否最近
        auto area = roi.height * roi.width;
        cout << "area = " << area << endl;
        if(state_ == STATE_WAIT_ENTER)
        {
            if(target.rois[0].type == "green")
            {
                RobotCount.nStartCount++;
                cout<< "green" << endl;
            }
            if(RobotCount.nStartCount>20)
            {
                state_ = STATE_STRAIGHT;
            }
        }
        // * 直道 -- 识别三个兵营旗帜
        if (state_ == STATE_STRAIGHT)
        {
            if (area > 14000 && (target.rois[0].type != "chu" && target.rois[0].type != "han"))
            {
                RobotFlags.bFlagFound = true;
                strFlag = target.rois[0].type;
            }
        }
        // * 弯道 -- 识别粮仓旗帜
        if(state_ == STATE_ROTATE)
        {
            if(area > 10000 &&(target.rois[0].type == "chu" ||target.rois[0].type == "han"))
            {
                RobotFlags.bgranaryFound = true;
                strFlag = target.rois[0].type;
            }
        }
#ifdef YOLO_DEBUG
        RCLCPP_INFO(this->get_logger(),
                    "Target %zu: Type: %s, height=%d, width=%d, conf=%.2f",
                    num,
                    target.rois[0].type.c_str(),
                    roi.height,
                    roi.width,
                    target.rois[0].confidence);
#endif
    }
}
#endif
void RosMainNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    const auto &orientation = msg->orientation;

    tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // RCLCPP_INFO(this->get_logger(), "Yaw: %f radians", yaw);

    RobotRun.currentYaw = yaw * 180.0 / M_PI;
    // RCLCPP_INFO(this->get_logger(), "Yaw: %f degrees", RobotRun.currentYaw);
}

/// @brief 巡线摄像头回调
/// @param msg 
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
    // ! 找球模式
    if (ImageState_ == IMAGE_STATE_FIND_BALL)
    {

        // * 将图像转换为 HSV 色彩空间
        cv::Mat imgOriginal = cv_ptr->image;
        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);
        // ! 直方图均衡化 效果不好注释
        // vector<cv::Mat> hsvSplit;
        // split(hsv_image, hsvSplit);
        // equalizeHist(hsvSplit[2], hsvSplit[2]);
        // merge(hsvSplit, hsv_image);

        // * 阈值操作，获取红色区域
        cv::Mat mask;
        cv::inRange(hsv_image, CvThreshold.Ball_lower, CvThreshold.Ball_upper, mask);

        // ! 图像开闭运算 效果不好注释
        // cv::Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
        // morphologyEx(mask, mask, MORPH_OPEN, element);
        // morphologyEx(mask, mask, MORPH_CLOSE, element);
        int nTargetX = 0;
        int nTargetY = 0;
        int nPixCount = 0;
        int nImgWidth = mask.cols;
        int nImgHeight = mask.rows;
        // * 遍历图像找到红色像素点
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
        // * 红色像素点阈值 > 200进入跟随
        // TODO 考虑修改为常量或参数
        if (nPixCount > 3000)
        {
            nTargetX /= nPixCount;
            nTargetY /= nPixCount;


#ifdef Desktop
            // * 画出球的中心
            Point line_begin = Point(nTargetX - 10, nTargetY);
            Point line_end = Point(nTargetX + 10, nTargetY);
            line(imgOriginal, line_begin, line_end, Scalar(255, 0, 0), 3);
            line_begin.x = nTargetX;
            line_begin.y = nTargetY - 10;
            line_end.x = nTargetX;
            line_end.y = nTargetY + 10;
            line(imgOriginal, line_begin, line_end, Scalar(255, 0, 0), 3);
#endif
            // * 简略的PID控制
            fVelFoward = (360 - nTargetY) * 0.002;
            fVelTurn = (320 - nTargetX) * 0.007;
            fVelTurnCount += fVelTurn;
#ifdef DEBUG
            printf("Target (%d, %d) PixelCount = %d\n", nTargetX, nTargetY, nPixCount);
            printf("fVelFoward:%f ", fVelFoward);
            printf(" fVelTurn:%f  ", fVelTurn);
#endif
            // geometry_msgs::msg::Twist vel_cmd;

            // vel_cmd.linear.x = fVelFoward;
            // vel_cmd.angular.z = fVelTurn;
            // ! 丑陋的滤波 转角过小时取消转向
            // TODO 考虑更改
            // auto nVelForwardStable = 0;
            // auto nVelTurnStable = 0;
            if (fVelFoward >= -0.05 && fVelFoward <= 0.05)
            {
                fVelFoward = 0;
                nVelForwardStable++;
            }
            else
            {
                nVelForwardStable = 0;
            }
            if (fVelTurn <= 0.05 && fVelTurn >= -0.05)
            {
                fVelTurn = 0;
                nVelTurnStable++;
            }
            else
            {
                nVelTurnStable = 0;
            }
            if (nVelForwardStable > 30 && nVelTurnStable > 30)
            {
                RobotFlags.bBallStable = true;
            }

            SetSpeed(fVelFoward, fVelTurn);
        }
        else
        {
            // * 进入else则停车
            // ! 此部分逻辑不够完善 没有找红球的行为
            // TODO 添加寻找红球行为
            // FindBall(); 记录下上一次的转向速度 寻找小球的方向
            RobotFlags.bBallFound = false;
#ifdef DEBUG
            printf("Target disappeared...\n");
            SetSpeed(0.15,0);
#endif
            // vel_cmd.linear.x = 0;
            // vel_cmd.linear.y = 0;
            // vel_cmd.linear.z = 0;
            // vel_cmd.angular.x = 0;
            // vel_cmd.angular.y = 0;
            // vel_cmd.angular.z = 0;
            // fVelFoward = 0;
            // fVelTurn = 0;
            // SetSpeed(fVelFoward, fVelTurn);
        }
        // twist_publisher_->publish(vel_cmd);
#ifdef Desktop
        imshow("Result", mask);
        imshow("RGB", imgOriginal);
        cv::waitKey(5);
#endif
        // ! 此部分效果不好 注释
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
// * 巡线
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

        // * 存储每一行的中线位置
        vector<int> leftBoundaries(rows, -1);
        vector<int> rightBoundaries(rows, -1);
        vector<int> middle(rows, 0);
        vector<int> auto_middle(rows, 0);
        float rows_area = 0.30;
        // * 第0行的中线从中间开始（假设为列数的一半）
        middle[rows - 20] = cols / 2 + 40; // == 420行
        auto_middle[rows] = cols / 2;
        for (int y = rows - 20; y >= int(rows * rows_area); --y)
        {
            int leftBoundary = -1, rightBoundary = -1;
            // * 使用上一行的中线位置来决定当前行的扫描起点
            int startX = middle[y + 1];
            //cout << "startX: " << startX << endl;
            // * 向左扫描，寻找左边界
            for (int z = startX; z > 6; --z)
            {
                uchar pixel = binary.at<uchar>(y, z);
                if ((pixel == 255 && binary.at<uchar>(y, z - 1) == 0 && binary.at<uchar>(y, z - 2) == 0 && binary.at<uchar>(y, z - 3) == 0 &&
                binary.at<uchar>(y, z - 4) == 0 && binary.at<uchar>(y, z - 5) == 0) || z == 7)
                {
                    leftBoundary = z;
                    //cout << "leftBoundary: " << leftBoundary << endl;
                    break;
                }
            }

            // * 向右扫描，寻找右边界
            for (int x = startX; x < cols - 6; ++x)
            {
                uchar pixel = binary.at<uchar>(y, x);
                if ((pixel == 255 && binary.at<uchar>(y, x + 1) == 0 && binary.at<uchar>(y, x + 2) == 0 && binary.at<uchar>(y, x + 3) == 0 &&
                binary.at<uchar>(y, x + 4) == 0 && binary.at<uchar>(y, x + 5) == 0) || x == cols - 7)
                {
                    rightBoundary = x;
                   // cout << "rightBoundary: " << rightBoundary << endl;
                    break;
                }
            }
            // * 记录下两个边界值
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
            // ! 画线函数 帧率过低 废弃
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
            if(auto_middle[y] != 0)
            {
                circle(cv_ptr->image, Point(auto_middle[y], y), 2, Scalar(0, 255, 0), -1);
            }
            cv::line(cv_ptr->image, cv::Point(cols / 2 + 40, 0),    // *画面一半的顶部
                                    cv::Point(cols / 2 + 40, rows), // *画面一半的底部
                                    cv::Scalar(255, 0, 0), 1); // *蓝色
            cv::line(cv_ptr->image, cv::Point(0, 300),         // *300行的左边
                                    cv::Point(cols, 300),      // *300行的右边
                                    cv::Scalar(255, 0, 0), 1); // *蓝色
            cv::line(cv_ptr->image, cv::Point(0, 260),         // *260行的左边
                                    cv::Point(cols, 260),      // *260行的右边
                                    cv::Scalar(255, 0, 0), 1); // *蓝色
            cv::line(cv_ptr->image, cv::Point(0, rows - 60),         // *260行的左边
                                    cv::Point(cols, rows - 60),      // *260行的右边
                                    cv::Scalar(255, 0, 0), 1); // *蓝色
        }

        imshow("Result Image", cv_ptr->image);

        // 等待用户按键，按任意键关闭窗口
        waitKey(1);
#endif
// * 控制部分
        float err = 0;
        float weight_count = 0;
        for(int i = 300;i<=459;i++)
        {
            err += middle[i] * Weight[i - 300];
            weight_count += Weight[i - 300];
        }
        err = err / weight_count;
        float middle_x = err; 
        // * 获取中线位置，并计算偏移量
        //int middle_x = middle[300];
        cout << "middle_x: " << middle_x << endl;
        float middle_offset = middle_x - (cols / 2 + 40);//360
        PIDControl.error = middle_offset;
        float speedX = 0.25;
        float speedZ = 0;
        float auto_kp = Update_kp_Speed(PIDControl.error, 20, 110);
        // float auto_kp = PIDControl.kp1;
        RCLCPP_INFO(this->get_logger(), "Middle Offset: %f", middle_offset);
        // * 转向逻辑
        if (middle_offset != 0)
        {
            speedZ = PIDControl.error * auto_kp +
                     (PIDControl.error - PIDControl.last_error) * PIDControl.kd1;
#ifdef DEBUG
            cout << "speedZ: " << speedZ << endl;
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
        // * 更新上一次的误差
        PIDControl.last_error = PIDControl.error;
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
                //RCLCPP_INFO(this->get_logger(), "Detected green area near yellow block with %d green pixels.", green_pixel_count);
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
                //RCLCPP_INFO(this->get_logger(), "Detected blue area near yellow block with %d blue pixels.", blue_pixel_count);
            }
        }

        // 显示处理后的图像
        imshow("Detected Image", cv_ptr->image);
        cv::waitKey(1); // 每1ms刷新一次显示
#endif
    }
}
/**********************************************************/
/*                     机器功能函数                         */
/**********************************************************/

/// @brief 机器说话
/// @param strToSpeak
void RosMainNode::Speak(std::string strToSpeak)
{
    std_msgs::msg::String voice_msg;
    voice_msg.data = strToSpeak;
    robot_voice_pub_->publish(voice_msg);
}

/// @brief 速度控制函数
/// @param lineX
/// @param angZ
void RosMainNode::SetSpeed(float lineX, float angZ)
{
    // ! angZ > 0 左转
    // ! angZ < 0 右转
    // auto twist = geometry_msgs::msg::Twist();
    geometry_msgs::msg::Twist twist;
    if (lineX >= RobotRun.fMaxSpeed)
        lineX = RobotRun.fMaxSpeed;
    else if (lineX <= -RobotRun.fMaxSpeed)
        lineX = -RobotRun.fMaxSpeed;

    if (angZ >= RobotRun.fMaxTurn)
        angZ = RobotRun.fMaxTurn;
    else if (angZ <= -RobotRun.fMaxTurn)
        angZ = -RobotRun.fMaxTurn;
    // auto twist = std::make_shared<Twist>();
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
    cout << "进入找球行为" << endl;
    auto message = geometry_msgs::msg::Twist();

    if (RobotFlags.bBallFound == true)
    {
        message.angular.z = -1.0 * (RobotRun.targetPoint - 480) / 300.0;
        message.linear.x = 0.24;
    }
    else
    {
        message.angular.z = 0;
        message.linear.x = 0.15;
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
void RosMainNode::ProcessCamp(const std::string &strFlag)
{
    std::map<std::string, std::string> campMessages = {
        {"qibing", "到达骑兵营"},
        {"bubing", "到达步兵营"},
        {"zhanche", "到达战车营"},
        {"han", "到达粮仓"},
        {"chu", "到达粮仓"}};

    auto it = campMessages.find(strFlag);
    if (it != campMessages.end())
    {
        if (strFlag == "qibing" && !CampFlags.cavalryCampArrived)
        {
            cout << "骑兵营已到达" << endl;
            CampFlags.cavalryCampArrived = true;
            RobotCount.nFlagCount++;
            Speak(it->second);
        }
        if (strFlag == "bubing" && !CampFlags.infantryCampArrived)
        {
            cout << "步兵营已到达" << endl;
            CampFlags.infantryCampArrived = true;
            RobotCount.nFlagCount++;
            Speak(it->second);
        }
        if (strFlag == "zhanche" && !CampFlags.tankCampArrived)
        {
            cout << "战车营已到达" << endl;
            CampFlags.tankCampArrived = true;
            RobotCount.nFlagCount++;
            Speak(it->second);
        }
        if((strFlag == "han" || strFlag == "chu") && state_ == STATE_ROTATE)
        {
            cout << "粮仓已到达" << endl;
            Speak(it->second);
            RobotFlags.bgranaryFound = true;

        }
    }
}

void RosMainNode::PID_Choose(int cmd)
{
    if(cmd == 1)
    {
        PIDControl.kp = PIDControl.kp2;
        PIDControl.ki = PIDControl.ki2; 
        PIDControl.kd = PIDControl.kd2;
        cout << "[PID]变更为直道PID参数" << endl;
    }
    else if(cmd == 2)
    {
        PIDControl.kp = PIDControl.kp1;
        PIDControl.ki = PIDControl.ki1;
        PIDControl.kd = PIDControl.kd1;
        cout << "[PID]变更为弯道PID参数" << endl;
    }
}

/// @brief PID-Kp参数更新
/// @param error
/// @param absmin
/// @param absmax
/// @return Kp参数
// ! 未进行弯道和直道判断 考虑最小二乘法
// TODO 优化该函数
float RosMainNode::Update_kp_Speed(int error, int absmin, int absmax)
{
    if (abs(error) < absmin)
        return 0; // 直道
    else if (abs(error) > absmax)
        return PIDControl.kp1; // 弯道
    else
        return PIDControl.kp1; // 默认
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
