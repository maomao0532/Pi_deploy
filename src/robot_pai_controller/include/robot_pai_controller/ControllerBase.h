#pragma once

// ros
#include <controller_manager_msgs/SwitchController.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>

#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Joy.h>

// std
#include <atomic>
#include <thread>

// Eigen
#include <Eigen/Dense>

// onnx
#include <onnxruntime/onnxruntime_cxx_api.h>

// user
#include "robot_pai_hw/hardware_interface/HybridJointInterface.h"
#include "robot_pai_controller/Types.h"


using namespace Eigen;

namespace robot_pai {

struct RobotCfg {
    struct ControlCfg {
        vector_t stiffness;
        vector_t damping;
        double actionScale;
        int decimation;
        double user_torque_limit;
        double user_power_limit;

        double stiffness_hip_yaw;
        double stiffness_hip_roll;
        double stiffness_hip_pitch;
        double stiffness_knee_pitch;
        double stiffness_ankle_pitch;
        double stiffness_ankle_roll;

        double damping_hip_yaw;
        double damping_hip_roll;
        double damping_hip_pitch;
        double damping_knee_pitch;
        double damping_ankle_pitch;
        double damping_ankle_roll;
    };

    struct InitState {
        // default joint angles
        double l_hip_pitch_joint;
        double l_hip_roll_joint;
        double l_hip_yaw_joint;
        double l_knee_pitch_joint;
        double l_ankle_pitch_joint;
        double l_ankle_roll_joint;

        double r_hip_pitch_joint;
        double r_hip_roll_joint;
        double r_hip_yaw_joint;
        double r_knee_pitch_joint;
        double r_ankle_pitch_joint;
        double r_ankle_roll_joint;
    };
    
    struct ObsScales {
        double linVel;
        double angVel;
        double dofPos;
        double dofVel;
        double quat;
        double heightMeasurements;
    };

    double clipActions;
    double clipObs;

    InitState initState;
    ObsScales obsScales;
    ControlCfg controlCfg;
};

struct JoyInfo {
  float axes[8];
  int buttons[11];
};

struct Proprioception {
  VectorXd jointPos;
  VectorXd jointVel;
  Vector3d baseAngVel;
  // vector3_t baseAngZyx;  // base angular pos eular (zyx)
  // quaternion_t baseAngQuat;  // base angular pos quat (zyx)
  VectorXd projectedGravity;
  Vector3d euler_xyz;
};

struct Command {
  std::atomic<double> x;
  std::atomic<double> y;
  std::atomic<double> yaw;
};

class ControllerBase : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface> {
public:
    enum class Mode : uint8_t { LIE, STAND, WALK };
    ControllerBase() = default;
    virtual ~ControllerBase() = default;
    // RobotHW中存有各种interface，interface中存有handle，handle中存有state和cmd
    virtual bool init(hardware_interface::RobotHW* robotHw, ros::NodeHandle& controllerNH);
    virtual void starting(const ros::Time& time);
    virtual void update(const ros::Time& time, const ros::Duration& period);

    virtual void handleLieMode();
    virtual void handleStandMode();
    virtual void handleWalkMode(){};

    virtual bool loadModel(ros::NodeHandle& nh) { return false; };
    virtual bool loadRLCfg(ros::NodeHandle& nh) { return false; };
    virtual void computeActions(){};
    virtual void computeObservation(){};
    
protected:
    virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);
    virtual void cmdVelCallback(const geometry_msgs::Twist& msg);
    std_msgs::Float64MultiArray createFloat64MultiArrayFromVector(const VectorXd& data);

    Mode mode_;
    int64_t loopCount_;
    Command command_;
    ros::Time startTime_;

    RobotCfg robotCfg_{};
    std::atomic_bool emergency_stop{false};
    std::atomic_bool start_control{false};

    VectorXd rbdState_;
    VectorXd measuredRbdState_;
    Proprioception propri_;

    // hardware interface
    std::vector<HybridJointHandle> hybridJointHandles_;
    hardware_interface::ImuSensorHandle imuSensorHandle_;

    ros::Subscriber startCtrlSub_;
    ros::Subscriber cmdVelSub_;
    ros::Subscriber emgStopSub_;
    controller_manager_msgs::SwitchController switchCtrlSrv_;
    ros::ServiceClient switchCtrlClient_;

    int actuatedDofNum_ = 12;

    ros::Publisher realJointPosPublisher_;              // 实际关节位置
    ros::Publisher rlJointPosPublisher_;                // 期望关节位置
    ros::Publisher realJointTauPublisher_;              // 实际力矩
    ros::Publisher rlJointTauPublisher_;                // 期望力矩

    double t_;
    
private:
    // PD stand
    std::vector<double> initJointAngles_;
    VectorXd standJointAngles_;
    geometry_msgs::Twist prev_command_;

    double standPercent_;
    double standDuration_;
};


}