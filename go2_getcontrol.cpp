#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>
#include <unitree/robot/b2/sport/sport_client.hpp>
using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;
using namespace unitree::robot::b2;

int main()
{
    unitree::robot::ChannelFactory::Instance()->Init(0, "eth0");//enx00e04c36141b为网口号，用户根据自身情况修改
    unitree::robot::go2::ObstaclesAvoidClient sc;
    sc.SetTimeout(5.0f);
    sc.Init();
    
    sc.UseRemoteCommandFromApi(false);//拿回手柄的控制权
    return 0;
}