0. 将机器人的两根usb线接到电脑上,赋予串口权限
sudo chmod -R 777 /dev/tty*
1. 编译
catkin build -DCMAKE_BUILD_TYPE=Release
2. 运行硬件launch
source ./devel/setup.bash
roslaunch  robot_pai_hw robot_pai_hw.launch
3. 运行控制器
roslaunch robot_pai_controller load_controller_pai.launch

policy_zero_jointpos.onnx  : 上下地形，不带phase
policy_422.onnx  : 平地，带phase
