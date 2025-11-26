#include "robot_pai_controller/ControllerBase.h"
#include <string.h>
#include <pluginlib/class_list_macros.hpp>
#include "robot_pai_controller/RotationTools.h"
#include <kdl_parser/kdl_parser.hpp>

namespace robot_pai {
bool ControllerBase::init(hardware_interface::RobotHW* robotHw, ros::NodeHandle& controllerNH) {
    // Hardware interface
    // 和URDF 以及 policy的顺序对应，各handle的对应在HW中实现
    std::vector<std::string> jointNames{"l_hip_pitch_joint", "l_hip_roll_joint", "l_hip_yaw_joint",
                                        "l_knee_pitch_joint", "l_ankle_pitch_joint", "l_ankle_roll_joint",
                                        "r_hip_pitch_joint", "r_hip_roll_joint", "r_hip_yaw_joint",
                                        "r_knee_pitch_joint", "r_ankle_pitch_joint", "r_ankle_roll_joint",};

    std::vector<std::string> footNames{"left_foot", "right_foot"};
    actuatedDofNum_ = jointNames.size();
    
    // Load policy model and rl cfg
    if (!loadModel(controllerNH)) {
        ROS_ERROR_STREAM("[RLControllerBase] Failed to load the model. Ensure the path is correct and accessible.");
        return false;
    }
    if (!loadRLCfg(controllerNH)) {
        ROS_ERROR_STREAM("[RLControllerBase] Failed to load the rl config. Ensure the yaml is correct and accessible.");
        return false;
    }

    standJointAngles_.resize(actuatedDofNum_);
    auto& initState = robotCfg_.initState;
    standJointAngles_ << initState.l_hip_pitch_joint, initState.l_hip_roll_joint, initState.l_hip_yaw_joint,
                         initState.l_knee_pitch_joint, initState.l_ankle_pitch_joint, initState.l_ankle_roll_joint,
                         initState.r_hip_pitch_joint, initState.r_hip_roll_joint, initState.r_hip_yaw_joint,
                         initState.r_knee_pitch_joint, initState.r_ankle_pitch_joint, initState.r_ankle_roll_joint;

    // 按jointNames顺序加载Handle
    auto* hybridJointInterface = robotHw->get<HybridJointInterface>();
    for (const auto& jointName : jointNames) {
        hybridJointHandles_.push_back(hybridJointInterface->getHandle(jointName));
    }
    imuSensorHandle_ = robotHw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");
    prev_command_.linear.x = 0.;
    prev_command_.linear.y = 0.;
    prev_command_.angular.z = 0.;

    // 话题订阅控制信号
    auto startControlCallback = [this](const std_msgs::Float32::ConstPtr& msg) {start_control = true; ROS_INFO("Start Control");};
    startCtrlSub_ = controllerNH.subscribe<std_msgs::Float32>("/start_control", 1, startControlCallback);
    // cmd
    cmdVelSub_ = controllerNH.subscribe("/cmd_vel", 1, &ControllerBase::cmdVelCallback, this);
    switchCtrlClient_ = controllerNH.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    auto emergencyStopCallback = [this](const std_msgs::Float32::ConstPtr& msg) {emergency_stop = true; ROS_INFO("Emergency Stop");};
    emgStopSub_ = controllerNH.subscribe<std_msgs::Float32>("/emergency_stop", 1, emergencyStopCallback);

    realJointPosPublisher_ = controllerNH.advertise<std_msgs::Float64MultiArray>("/data_analysis/real_joint_pos", 1);
    rlJointPosPublisher_ = controllerNH.advertise<std_msgs::Float64MultiArray>("/data_analysis/desire_joint_pos", 1);
    realJointTauPublisher_ = controllerNH.advertise<std_msgs::Float64MultiArray>("/data_analysis/real_torque", 1);
    rlJointTauPublisher_ = controllerNH.advertise<std_msgs::Float64MultiArray>("/data_analysis/desire_torque", 1);
    return true;
}

void ControllerBase::starting(const ros::Time& time) {
    startTime_ = ros::Time::now();
    updateStateEstimation(time, ros::Duration(0.002));

    initJointAngles_.resize(hybridJointHandles_.size());
    for (int i = 0; i <hybridJointHandles_.size(); i++) {
        initJointAngles_[i] = hybridJointHandles_[i].getPosition();
    }

    double durationSecs = 2.0;
    standDuration_ = durationSecs * 1000.0;
    standPercent_  = 0;
    mode_ = Mode::LIE;
    loopCount_ = 0;
}

void ControllerBase::update(const ros::Time& time, const ros::Duration& period) {
    updateStateEstimation(time, period);
    switch (mode_) {
        case Mode::LIE:
            handleLieMode();
            break;
        case Mode::STAND:
            handleStandMode();
            break;
        case Mode::WALK:
            handleWalkMode();
            break;
        default:
            ROS_ERROR_STREAM("Unexpected mode encountered: " << static_cast<int>(mode_));
            break;
    }

    if (emergency_stop) {
        for (size_t j = 0; j < hybridJointHandles_.size(); ++j)
            hybridJointHandles_[j].setCommand(0, 0, 0, 0, 0);
    }
    if (emergency_stop && start_control) {
        emergency_stop = false;
        starting(time);
        start_control = false;
    }

    loopCount_++;


}

void ControllerBase::handleLieMode() {
    if (standPercent_ < 1) {
        for (int j = 0; j < hybridJointHandles_.size(); j++) {
            double pos_des = initJointAngles_[j] * (1 - standPercent_) + standJointAngles_[j] * standPercent_;
            hybridJointHandles_[j].setCommand(pos_des, 0, 10, 0.25, 0);
            // hybridJointHandles_[j].setCommand(pos_des, 0, 80, 2, 0);
            // ROS_WARN("123");
        }
        standPercent_ += 1 / standDuration_;
    } else {
        mode_ = Mode::STAND;
    }
}

void ControllerBase::handleStandMode() {
    if (loopCount_ > 3000 && start_control) {
        startTime_ = ros::Time::now();
        mode_ = Mode::WALK;
    }
}

void ControllerBase::updateStateEstimation(const ros::Time& time, const ros::Duration& period) {
    // q 先左后右
    VectorXd jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size()), jointTor(hybridJointHandles_.size()),
             jointDesirePos(hybridJointHandles_.size());
    quaternion_t quat;
    quat.setIdentity();
    Vector3d angularVel, linearAccel;
    Matrix3d orientationCovariance, angularVelCovariance, linearAccelCovariance;
    for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
        jointPos(i) = hybridJointHandles_[i].getPosition();
        jointVel(i) = hybridJointHandles_[i].getVelocity();
        jointTor(i) = hybridJointHandles_[i].getEffort();
        jointDesirePos(i) = hybridJointHandles_[i].getPositionDesired();
    }
    for (size_t i = 0; i < 4; ++i) {
        quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];   // xyzw
    }
    for (size_t i = 0; i < 3; ++i) {
        angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
        // linearAccel(i) = baseSensorHandle_.getLinearAcceleration()[i];
    }

    propri_.jointPos = jointPos;
    propri_.jointVel = jointVel;
    propri_.baseAngVel = angularVel;
    
    Vector3d gravityVector(0, 0, -1);
    Vector3d zyx = quatToZyx(quat);
    matrix_t inverseRot = getRotationMatrixFromZyxEulerAngles(zyx).inverse();
    propri_.projectedGravity = inverseRot * gravityVector;
    // ROS_WARN("2");
    propri_.euler_xyz = zyx.reverse();

    
    // ROS_WARN("4");

    if (loopCount_ > 3001) {
        t_ = (time - startTime_).toSec();
    }      
    
    VectorXd jointDesireTau(12);
    jointDesireTau = robotCfg_.controlCfg.stiffness.cwiseProduct(jointDesirePos - jointPos) - robotCfg_.controlCfg.damping.cwiseProduct(jointVel);
    // jointDesireTau = 10 * (jointDesirePos - jointPos) - 0.25 * (jointVel);
    
    realJointPosPublisher_.publish(createFloat64MultiArrayFromVector(jointPos));
    rlJointPosPublisher_.publish(createFloat64MultiArrayFromVector(jointDesirePos));
    realJointTauPublisher_.publish(createFloat64MultiArrayFromVector(jointTor));
    rlJointTauPublisher_.publish(createFloat64MultiArrayFromVector(jointDesireTau));
    
}

void ControllerBase::cmdVelCallback(const geometry_msgs::Twist& msg) {
    double max_change_rate_ = 1;
    double dt = 0.1; // Assuming a fixed time step of 0.1s (10Hz)
    double diff_x, diff_y, diff_yaw;
  // Calculate the difference between the new command and the previous command
    diff_x = msg.linear.x - prev_command_.linear.x;
    diff_y = msg.linear.y - prev_command_.linear.y;
    diff_yaw = msg.angular.z - prev_command_.angular.z;
  

  // Limit the change rate
  if (std::abs(diff_x) > max_change_rate_ * dt) {
    diff_x = (diff_x > 0 ? 1 : -1) * max_change_rate_ * dt;
  }
  if (std::abs(diff_y) > max_change_rate_ * dt) {
    diff_y = (diff_y > 0 ? 1 : -1) * max_change_rate_ * dt;
  }
  if (std::abs(diff_yaw) > max_change_rate_ * dt) {
    diff_yaw = (diff_yaw > 0 ? 1 : -1) * max_change_rate_ * dt;
  }

  // Update the command
  command_.x = prev_command_.linear.x + diff_x;
  command_.y = prev_command_.linear.y + diff_y;
  command_.yaw = prev_command_.angular.z + diff_yaw;
//   ROS_WARN("%.3f, %.3f, %.3f",prev_command_.linear.x + diff_x, prev_command_.linear.y + diff_y, prev_command_.angular.z + diff_yaw);

    prev_command_.linear.x = command_.x;
  prev_command_.linear.y = command_.y;
  prev_command_.angular.z = command_.yaw;
}

std_msgs::Float64MultiArray ControllerBase::createFloat64MultiArrayFromVector(const VectorXd & data) {
    std_msgs::Float64MultiArray msg;
    msg.data.resize(data.size());
    // copy data from eigen vector to std vector
    VectorXd::Map(&msg.data[0], data.size()) = data;
    return msg;
}

}

PLUGINLIB_EXPORT_CLASS(robot_pai::ControllerBase, controller_interface::ControllerBase)