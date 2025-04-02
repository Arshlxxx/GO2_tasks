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

  void move()
  {
    sport_client.Move(0.5, 0, 0); // 执行运动命令
  }

  void stop()
  {
    sport_client.StopMove(); // 停止运动
    mQuit = true; // 设置线程退出标志
  }

  void GetInitState()
  {
    px0 = state.position()[0];
    py0 = state.position()[1];
    yaw0 = state.imu_state().rpy()[2];
    std::cout << "initial position: x0: " << px0 << ", y0: " << py0 << ", yaw0: " << yaw0 << std::endl;
  }

  void HighStateHandler(const void *message)
  {
    state = *(unitree_go::msg::dds_::SportModeState_ *)message;
  }

  unitree_go::msg::dds_::SportModeState_ state;
  unitree::robot::go2::SportClient sport_client;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;

  double px0, py0, yaw0;
  double ct = 0;
  int flag = 0;
  float dt = 0.005;
  bool mQuit = false; // 线程退出标志
};

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  Custom custom;

  sleep(1); // 等待1秒，确保初始状态稳定

  custom.GetInitState(); // 获取初始位置

  // 创建一个周期性线程
  unitree::common::ThreadPtr threadPtr = unitree::common::CreateRecurrentThread(custom.dt * 1000000, std::bind(&Custom::move, &custom));

  sleep(3);  // 让机器人走动3秒

  custom.stop();  // 停止运动，并设置退出标志
  std::cout << "down" << std::endl;

  // 等待线程完成
  threadPtr->Join();  // 等待线程退出

  return 0;
}
