#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;

void RotateToFindBall(unitree::robot::go2::ObstaclesAvoidClient &sc, float angular_velocity)
{
    std::cout << "开始旋转寻找球..." << std::endl;
    // 让机器人旋转一圈
    while (true)
    {
        sc.Move(0.0, 0.0, angular_velocity); // 向右旋转
        usleep(100000);  // 每100ms更新一次

        // 可以根据某种条件停止旋转，比如检测到球的位置
        // 例如：如果机器人已经旋转了一定角度，或者传感器检测到球
        // 这里为了演示我们做一个简化的条件，假设旋转360度后停止
        static int angle_counter = 0;
        angle_counter++;
        if (angle_counter >= 72) // 每次旋转5度，总共旋转360度
        {
            std::cout << "旋转结束，停止旋转" << std::endl;
            sc.Move(0.0, 0.0, 0.0); // 停止旋转
            break;
        }
    }
}

int main()
{
    unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");//enx00e04c36141b为网口号，用户根据自身情况修改
    unitree::robot::go2::ObstaclesAvoidClient sc;

    double radius = 0.3;  // 半径
    double angular_velocity = 0.2;  // 角速度，单位：rad/s
    double linear_velocity = radius * angular_velocity;  // 线速度，计算公式：v = r * ω

    sc.SetTimeout(5.0f);
    sc.Init();
    // sc.SwitchSet(true);//开启避障
    usleep(1000000);
    sc.UseRemoteCommandFromApi(true);//抢夺遥控器的速度控制权

    // sc.Move(0.0,linear_velocity, -angular_velocity);//沿着机体x方向以1m/s的速度运动，碰到障碍物自动避障
    
    // int tem = 0;
    // while (true)
    // {
    //   usleep(1000000);
    //   if(tem<=10)
    //   {
    //     tem++;
    //   }
    //   else
    //   {
    //     sc.UseRemoteCommandFromApi(false);//5秒后关闭
    //     break;
    //     // sc.SwitchSet(false);//5秒后关闭
    //   }
    // }
    RotateToFindBall(sc, 0.4);  // 设置旋转角速度，0.4可以根据需要调整

    sc.UseRemoteCommandFromApi(false);//5秒后关闭
    return 0;
}