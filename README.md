# Pi Deploy

## 使用说明

### 0. 准备工作

将机器人的两根 USB 线接到电脑上，赋予串口权限：

```bash
sudo chmod -R 777 /dev/tty*
```

### 1. 编译

```bash
catkin build -DCMAKE_BUILD_TYPE=Release
```

### 2. 运行硬件 Launch

```bash
source ./devel/setup.bash
roslaunch robot_pai_hw robot_pai_hw.launch
```

### 3. 运行控制器

```bash
roslaunch robot_pai_controller load_controller_pai.launch
```

## 模型文件说明

| 文件名 | 说明 |
| --- | --- |
| `policy_zero_jointpos.onnx` | 上下地形，不带 phase |
| `policy_422.onnx` | 平地，带 phase |
