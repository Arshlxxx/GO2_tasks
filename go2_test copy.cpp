/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <cmath>


#include <iostream>
#include <fstream>  // 引入文件操作头文件
#include <vector>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/go2/obstacles_avoid/obstacles_avoid_client.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>


#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;

std::array<double, 3> ball_position = {100, 100, 100};
int camere_flag = 0;

class Custom
{
public:
  Custom()
  {
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);
  };

  // Get initial position
  std::array<double, 3> GetState()
{
    px0 = state.position()[0];
    py0 = state.position()[1];
    yaw0 = state.imu_state().rpy()[2];
    std::cout << " position: x0: " << px0 << ", y0: " << py0 << ", yaw0: " << yaw0 << std::endl;
    std::array<double, 3> current_state = {px0 , py0, yaw0};
    return current_state;
};

  void HighStateHandler(const void *message)
  {
    state = *(unitree_go::msg::dds_::SportModeState_ *)message;
  };

  unitree_go::msg::dds_::SportModeState_ state;
  unitree::robot::go2::SportClient sport_client;
  unitree::robot::go2::ObstaclesAvoidClient sc;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;

  double px0, py0, yaw0; // 初始状态的位置和偏航
  double ct = 0;         // 运行时间
  int flag = 0;          // 特殊动作执行标志
  float dt = 0.005;      // 控制步长0.001~0.01

  std::array<double, 3> diff_state = {0.0, 0.0, 0.0};
};

std::array<double, 2> ConvertToGlobalCoordinates(
    const std::array<double, 3> &current_state,  // 现在狗的位置
    const std::array<double, 2> &local_target)   // 球的位置
{
    double x_local = local_target[0];
    double y_local = local_target[1];
    double x_current = current_state[0];
    double y_current = current_state[1];
    double yaw = current_state[2];

    // 使用旋转矩阵将局部坐标转换为全局坐标
    double x_global = x_current + x_local * cos(yaw) - y_local * sin(yaw);
    double y_global = y_current + x_local * sin(yaw) + y_local * cos(yaw);

    return {x_global, y_global};
}

// 检查狗是否接近边界
bool IsDogNearBoundary(const std::array<double, 3>& current_state,const std::array<double, 3>& init_state, double x_limit, double y_limit, double boundary_limit)
{
    //绝对的
    double dx = current_state[0] - init_state[0];  // 获取当前位置相对于初始位置的 x 坐标（前进方向）
    double dy = current_state[1] - init_state[1];  // 获取当前位置相对于初始位置的 y 坐标（侧向

    // //用了里程计的
    // double dx = current_state[0];  // 获取当前位置相对于初始位置的 x 坐标（前进方向）
    // double dy = current_state[1];  // 获取当前位置相对于初始位置的 y 坐标（侧向

    // 判断狗是否接近场地边界
    if (std::abs(dx) <= boundary_limit || std::abs(dx) >= x_limit - boundary_limit ||
        std::abs(dy) >= y_limit - boundary_limit)
    {
        return true;  // 如果狗接近场地边界，返回 true
    }
    return false;  // 如果狗没有接近边界，返回 false
}

//视觉查看球的位置
void BallPositionCallback(const geometry_msgs::Point::ConstPtr& msg) {
    ball_position[0] = msg->x; // X 位置
    ball_position[1] = msg->y; // Y 位置
    ball_position[2] = msg->z; // Z 位置
    camere_flag = 1;
    ROS_INFO("Received Ball Position: x=%f, y=%f, z=%f", ball_position[0], ball_position[1], ball_position[2]);
}



int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }
    // 初始化ROS节点
  ros::init(argc, argv, "football_position_listener");
  ros::NodeHandle nh;

    //到时候是相机ros传递出 球和门的位置
  double target_px1 = 0.1;
  double target_py1 = 0.7;

  double door_px1 = 3;
  double door_py1= 2.4;

  int kick_flag = 0; //默认是往左边踢球（注意这个左右是基于狗的位置）

  double Boundary_x = 4.8; //狗的位置在场地下边线的正中间，x边界是0和该变量
  double Boundary_y = 2.15; //y的边界是该变量的正负
  double Boundary_limit = 0.6; //threshould

  int police_flag = 0; //判断策略1还是2

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  unitree::robot::go2::ObstaclesAvoidClient sc;
  unitree::robot::go2::SportClient sport_client;

  std::array<double, 3> current_state = {0.0, 0.0, 0.0};
  std::array<double, 3> init_state = {0.0, 0.0, 0.0};
  std::array<double, 2> global_target = {0 ,0};
  std::array<double, 2> local_target ={0 ,0};
  float vx = 0.3;
  float vy = 0.3;
  float vyaw = 0.7;
  int stage_flage = 1;

  //初始化
  sc.SetTimeout(5.0f);
  sc.Init();
  sport_client.SetTimeout(5.0f);
  sport_client.Init();

  Custom custom;

  // Subscribe to football position topic
  ros::Subscriber football_pos_sub = nh.subscribe("/football_position", 10, BallPositionCallback);
  sleep(1);
  init_state = custom.GetState();

  
  //转换世界坐标系 球
  local_target = {target_px1, target_py1}; // 局部目标点
  global_target = ConvertToGlobalCoordinates(init_state, local_target);
  // std::array<double, 3> target_state = {target_px1 + init_state[0], target_py1 + init_state[1], init_state[2]};
  std::array<double, 3> target_state = {global_target[0], global_target[1], init_state[2]};

  //同理转换门的坐标系
  local_target = {door_px1, door_py1};
  global_target = ConvertToGlobalCoordinates(init_state, local_target);
  std::array<double, 2> target_door = {global_target[0], global_target[1]};
  std::cout<< "target x: " << target_state[0]  << " Y: " << target_state[1]  << " yaw: " << target_state[2]   << std::endl;

  sc.UseRemoteCommandFromApi(true);//抢夺遥控器的速度控制权

  double radius = 0.55;  // 半径
  double angular_velocity = 0.4;  // 角速度，单位：rad/s
  double linear_velocity = radius * angular_velocity;  // 线速度，计算公式：v = r * ω
  double yaw_diff_door = 100;

  sc.Move(0,0,vyaw);

  ros::Rate loop_rate(10);
  while (ros::ok())//0.ros回调主循环
  {
    ros::spinOnce();
    
    if(stage_flage == 1)
    {
          sc.Move(0,0,vyaw);//1.search
    }
    if(stage_flage == 2)
    {
          sc.Move(vx, 0.0, 0.0); 
    }
    if (kick_flag == 0 && stage_flage == 3)
    {
      
          //3.旋转射门
      current_state = custom.GetState();  // 获取当前状态
      // 计算与球门的相对位置
      double dx_door = target_door[0] - current_state[0];
      double dy_door = target_door[1] - current_state[1];
      double door_angle = atan2(dy_door, dx_door);  // 计算球门角度
      // 计算机器人当前角度和球门角度之间的差值
      double yaw_diff_door = current_state[2] - door_angle;
      // 将yaw差值标准化到[-π, π]范围
      if (yaw_diff_door > M_PI) yaw_diff_door -= 2 * M_PI;
      if (yaw_diff_door < -M_PI) yaw_diff_door += 2 * M_PI;
      std::cout << "偏航角误差" << yaw_diff_door << std::endl;

      if (yaw_diff_door > 0)
      {
          sc.Move(0.0,linear_velocity, -angular_velocity);//绕着球顺时针转，向右踢
      }
      else
      {
          // sc.Move(0,0,0);
          sc.Move(0.0,-linear_velocity, angular_velocity);//绕着球逆时针转,向左踢 
      }

      if (std::abs(yaw_diff_door) < 0.1 && stage_flage == 3)
      {
        std::cout << "射门准备完毕" << std::endl;
        stage_flage = 4;
        // v // 停止圆周运动
        // break;  // 退出圆周运动
      }
    }
    if (kick_flag == 1 && stage_flage == 3)
    {
      sc.Move(0.0,-linear_velocity, angular_velocity);//绕着球逆时针转,向左踢 
    }
    if (stage_flage == 4)
    {
        sc.Move(0.8, 0.0, 0.0);
        std::cout << "射门" << std::endl; 
        int tem = 0;
        while (true)
        {
          usleep(100000);
          if(tem<=18)
          {
            tem++;
          }
          else
          {
            sc.Move(0.0,0, -0);;
            std::cout << "射门完毕" << std::endl;
            return 0;
            break;
          }
        }
        break;
    }
    //1.找到足球
    if (std::abs(ball_position[0]) < 0.11 && stage_flage == 1)
    {
      std::cout << "找到足球" << std::endl;
      stage_flage = 2;
    }
    //2.走直线
    if(std::abs(ball_position[2] < 0.65 && stage_flage == 2))
    {
      std::cout << "接近足球" << std::endl;
      stage_flage = 3;
    }
    loop_rate.sleep();  // 等待直到下一个循环
  }

  return 0;

// 3. 圆周运动准备射门


  if (kick_flag == 0)
  {
     sc.Move(0.0,linear_velocity, -angular_velocity);//绕着球顺时针转，向右踢
  }
  else
  {
     sc.Move(0.0,-linear_velocity, angular_velocity);//绕着球逆时针转,向左踢 
  }

  // 计算球门的相对角度，确保机器人面向球门
  while (true)
  {
    usleep(100000);  // 每100ms更新一次
    current_state = custom.GetState();  // 获取当前状态

    if(IsDogNearBoundary(current_state, init_state, Boundary_x, Boundary_y, Boundary_limit))
    {
      std::cout<< "detact the Boundary "<< std::endl;
      police_flag = 1;
      break;
    };

    // 计算与球门的相对位置
    double dx_door = target_door[0] - current_state[0];
    double dy_door = target_door[1] - current_state[1];
    double door_angle = atan2(dy_door, dx_door);  // 计算球门角度

    // 计算机器人当前角度和球门角度之间的差值
    double yaw_diff_door = current_state[2] - door_angle;

    // 将yaw差值标准化到[-π, π]范围
    if (yaw_diff_door > M_PI) yaw_diff_door -= 2 * M_PI;
    if (yaw_diff_door < -M_PI) yaw_diff_door += 2 * M_PI;

    // 如果机器人面向球门（yaw差值接近0），停止圆周运动
    if (std::abs(yaw_diff_door) < 0.1)
    {
      sc.Move(0.0, 0.0, 0.0);  // 停止圆周运动
      break;  // 退出圆周运动
    }
  }


  if(police_flag == 0)      //4.kick策略1下的
  {
    sc.Move(0.8, 0.0, 0.0);
        int tem = 0;
        while (true)
        {
          usleep(100000);
          if(tem<=15)
          {
            tem++;
          }
          else
          {
            break;
          }
        }
  }
  else
  {
    //向左移动两步
    sc.Move(0.0, 0.3,0.0);
        int tem = 0;
        while (true)
        {
          usleep(100000); //0.7秒
          if(tem<=9)
          {
            tem++;
          }
          else
          {
            break;
          }
        }
      //向前走
      sc.Move(0.35, 0.0 ,0.0);
        int tem1 = 0;
        while (true)
        {
          usleep(100000);
          if(tem1<=17)
          {
            tem1++;
          }
          else
          {
            break;
          }
        }
        //向右走
         sc.Move(0.0, -0.5,0.0);
        int tem2 = 0;
        while (true)
        {
          usleep(100000);
          if(tem2<=15)
          {
            tem2++;
          }
          else
          {
            break;
          }
        }
  };
  // 完成圆周运动后停止
  sport_client.StopMove();
  sc.UseRemoteCommandFromApi(false);  // 释放遥控器的速度控制权

  return 0;
}
