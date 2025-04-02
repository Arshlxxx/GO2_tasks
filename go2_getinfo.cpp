#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::b2;

#include <cmath>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"
#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

class Custom
{    
public:
    Custom();

    void Init();
    void Start();
    
private:
    void InitLowCmd();
    void LowStateMessageHandler(const void* messages);
    void LowCmdWrite();
    int queryMotionStatus();
    std::string queryServiceName(std::string form, std::string name);

    void footballPosCallback(const geometry_msgs::Point::ConstPtr& msg);
    void HighStateHandler(const void *message);
    std::array<double, 3> GetState();
    std::array<double, 2> ConvertToLocalCoordinates(std::array<double, 3> current_state, std::array<double, 3> init_state);

private:
    float dt = 0.002; // 0.001~0.01
    double px0, py0, yaw0; // 初始状态的位置和偏航

    MotionSwitcherClient msc;

    unitree_go::msg::dds_::LowCmd_ low_cmd{};      // default init
    unitree_go::msg::dds_::LowState_ low_state{};  // default init

    unitree_go::msg::dds_::SportModeState_ state;
    unitree::robot::go2::SportClient sport_client;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

    /*LowCmd write thread*/
    ThreadPtr lowCmdWriteThreadPtr;

    ros::NodeHandle nh;  // NodeHandle for ROS
    ros::Subscriber football_pos_sub;  // Subscriber for football position
};

// 构造函数的定义
Custom::Custom()
{
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    // 订阅 /football_position 话题
    football_pos_sub = nh.subscribe("/football_position", 1000, &Custom::footballPosCallback, this);

    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);
}

void Custom::Init()
{
    /*create publisher*/
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);

    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);
}

std::array<double, 3> Custom::GetState()
{
    px0 = state.position()[0];
    py0 = state.position()[1];
    yaw0 = state.imu_state().rpy()[2];
    std::cout << " position: x0: " << px0 << ", y0: " << py0 << ", yaw0: " << yaw0 << std::endl;
    std::array<double, 3> current_state = {px0 , py0, yaw0};
    return current_state;
}

std::array<double, 2> Custom::ConvertToLocalCoordinates(std::array<double, 3> current_state, std::array<double, 3> init_state)
{
    // 计算当前位置相对初始位置的坐标差
    double dx = current_state[0] - init_state[0];  // 当前X与初始X的差值
    double dy = current_state[1] - init_state[1];  // 当前Y与初始Y的差值

    // 使用旋转矩阵根据偏航角将坐标转换到局部坐标系
    double local_x = dx * cos(-yaw0) - dy * sin(-yaw0);  // X轴方向的转换
    double local_y = dx * sin(-yaw0) + dy * cos(-yaw0);  // Y轴方向的转换
    std::cout << "dx:" << local_x << " dy:" << local_y << std::endl;
    // 返回局部坐标
    std::array<double, 2> local_coordinates = {local_x, local_y};
    return local_coordinates;
}

void Custom::Start()
{
    float base_velocity_x = 0.0f;
    float base_velocity_y = 0.0f;
    float base_velocity_z = 0.0f;

    float base_position_x = 0.0f;
    float base_position_y = 0.0f;
    float base_position_z = 0.0f;

    std::array<double, 3> current_state = {0.0, 0.0, 0.0};
    std::array<double, 3> init_state = {0.0, 0.0, 0.0};
    double dx,dy;

    // 记录时间
    auto last_time = std::chrono::high_resolution_clock::now();
    auto last_print_time = last_time;

    ros::Rate loop_rate(10);

    init_state = GetState();

    while (ros::ok())
    {
        ros::spinOnce();
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = current_time - last_time;
        float dt_actual = elapsed.count();

        // 更新 last_time
        last_time = current_time;

        // get base 线速度
        base_velocity_x = state.velocity()[0];
        base_velocity_y = state.velocity()[1];
        base_velocity_z = state.velocity()[2];

        base_position_x = state.position()[0];
        base_position_y = state.position()[1];
        base_position_z = state.position()[2];

        current_state = GetState();
        ConvertToLocalCoordinates(current_state, init_state);
        // 每秒打印一次
        if (std::chrono::duration<float>(current_time - last_print_time).count() >= 1.0f)
        {
            last_print_time = current_time;  // 更新最后打印时间
            // std::cout << " position: x0: " << base_position_x << ", y0: " << base_position_y << ", yaw0: " << state.imu_state().rpy()[2] << std::endl;
        }

        // 短暂休眠以避免高CPU占用
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void Custom::footballPosCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    // 提取 x, y, z 坐标
    float football_x = msg->x;
    float football_y = msg->y;
    float football_z = msg->z;

    // 在控制台打印
    std::cout << "Football Position -> x: " << football_x 
              << ", y: " << football_y 
              << ", z: " << football_z << std::endl;
}

void Custom::LowStateMessageHandler(const void* message)
{
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
}

void Custom::HighStateHandler(const void *message)
{
    state = *(unitree_go::msg::dds_::SportModeState_ *)message;
}

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1); 
    }

    std::cout << "WARNING: Make sure the robot is hung up or lying on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ChannelFactory::Instance()->Init(0, argv[1]);  

    ros::init(argc, argv, "robot_data_printer");

    Custom custom;

    custom.Init();
    custom.Start();

    return 0;
}
