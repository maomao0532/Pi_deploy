#pragma once

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>

// ROS
#include <ros/ros.h>
#include <urdf/model.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>

// ROS control
#include "robot_pai_hw/hardware_interface/HybridJointInterface.h"
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "robot_pai_hw/hardware/serial_struct.h"
#include "robot_pai_hw/hardware/motor/robot.h"
#include <Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>

using namespace Eigen;
using namespace std;


namespace robot_pai {

struct PaiMotorData {
    double pos_, vel_, tau_;                 // state
    double posDes_, velDes_, kp_, kd_, ff_;  // command
};

struct PaiImuData {
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};


class RobotPaiHW : public hardware_interface::RobotHW {
// HW中存放各种interface，interface中存放机器人状态和控制指令
public:
    RobotPaiHW() = default;
    // root_nh在调用者的命名空间的根位置创建的，robot_hw_nh在一个特定于 RobotHW 实例的命名空间下创建的
    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
    void read(const ros::Time& time, const ros::Duration& period);
    void write(const ros::Time& time, const ros::Duration& period) override;

private:
    bool setupJoints();
    bool setupImu();
    void imuCallBack(const sensor_msgs::ImuConstPtr& msg);

    // sub camera odom
    ros::Subscriber imu_sub_;
    std::mutex base_mutex_;
    std::shared_ptr<ros::AsyncSpinner> spinner_;

    // Interface
    hardware_interface::JointStateInterface jointStateInterface_;  // NOLINT(misc-non-private-member-variables-in-classes)
    hardware_interface::ImuSensorInterface imuSensorInterface_;
    HybridJointInterface hybridJointInterface_;                    // NOLINT(misc-non-private-member-variables-in-classes)

    PaiMotorData jointData_[12]{};
    PaiImuData imuData_{};
    
    // ros::Publisher realJointPosPublisher_;          // 实际位置
    // ros::Publisher realJointTauPublisher_;          // 实际力矩
    // ros::Publisher rlTauPublisher_;                 // 期望力矩

    std::atomic_bool test_torque{false}; 
    ros::Subscriber testSub_;
    ros::Subscriber imuSub_;

    std_msgs::Float64MultiArray createFloat64MultiArrayFromVector(const VectorXd& data);

    // pai hardware
    std::shared_ptr<livelybot_serial::robot> rb_;
    // 关节方向和偏置
    std::vector<int> jointAxis_;
    std::vector<double> jointBias_;

    // rviz 可视化用发布
    std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;
    tf::TransformBroadcaster tfBroadcaster_;
};

}  // namespace legged
