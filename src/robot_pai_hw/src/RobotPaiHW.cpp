#include "robot_pai_hw/RobotPaiHW.h"
#include <std_msgs/Float32.h>
#include <kdl_parser/kdl_parser.hpp>

namespace robot_pai {

bool RobotPaiHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
    registerInterface(&jointStateInterface_);
    registerInterface(&imuSensorInterface_);
    registerInterface(&hybridJointInterface_);
    
    setupJoints();
    setupImu();

    // 在Controller中进行发布
    // realJointPosPublisher_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("/data_analysis/real_joint_pos", 10);
    // realJointTauPublisher_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("/data_analysis/real_torque", 10);
    // rlTauPublisher_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("/data_analysis/desire_torque", 10);//发布RL输出的力矩
    
    spinner_ = std::make_shared<ros::AsyncSpinner>(3);      // 多线程处理回调函数

    // 初始化电机通信
    rb_ = std::make_shared<livelybot_serial::robot>();
    ros::Duration(1.0).sleep();
    for (motor *m : rb_->Motors) {   
        m->fresh_cmd_int16(0, 0, 0, 0.0, 0, 0, 0.0, 0, 0.0);
    }
    rb_->motor_send_2();

    jointAxis_.assign(12, 1);
    // jointAxis_ = {1, 1, -1, -1, -1, 1,
    //               1, -1, 1, -1, -1, -1};

    jointAxis_ = {1, 1, -1, -1, 1, 1,
                  1, -1, 1, -1, 1, -1};
    
    jointBias_.assign(12, 0.);
    jointBias_ = {0.0, 0.4, -0.65, 0.0, 0.0, 0.25,
                  0.0, 0.4, -0.65, 0.0, 0.0, 0.25};

    auto testCallback = [this](const std_msgs::Float32::ConstPtr& msg) {test_torque=true;};
    testSub_ = robot_hw_nh.subscribe<std_msgs::Float32>("/test_torque", 1, testCallback);

    imuSub_ = robot_hw_nh.subscribe<sensor_msgs::Imu>("/imu", 10, &RobotPaiHW::imuCallBack, this);
    spinner_->start();

    // 读取urdf
    urdf::Model urdfModel;
    if (!urdfModel.initParam("robot_description"))
    {
      std::cerr << "[LeggedRobotVisualizer] Could not read URDF from: \"legged_robot_description\"" << std::endl;
    }
    else
    {
      KDL::Tree kdlTree;
      kdl_parser::treeFromUrdfModel(urdfModel, kdlTree);
      robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
    }

    return true;
}

std_msgs::Float64MultiArray RobotPaiHW::createFloat64MultiArrayFromVector(const VectorXd& data)
{
  std_msgs::Float64MultiArray msg;
  msg.data.resize(data.size());
  // copy data from eigen vector to std vector
  VectorXd::Map(&msg.data[0], data.size()) = data;
  return msg;
}

void RobotPaiHW::read(const ros::Time& time, const ros::Duration& period) {
    // motor -> joint
    VectorXd motor_q(12), motor_dq(12), motor_tau(12);

    for (int i = 0; i < 12; i++) {
        motor_q(i) = rb_->Motors[i]->get_current_motor_state()->position * jointAxis_[i] + jointBias_[i];
        motor_dq(i) = rb_->Motors[i]->get_current_motor_state()->velocity * jointAxis_[i];
        motor_tau(i) = rb_->Motors[i]->get_current_motor_state()->torque * jointAxis_[i];
    }
    
    for (int i = 0; i < 12; i++) {
        jointData_[i].pos_ = motor_q(i);
        // jointData_[i].pos_ = rb_->Motors[i]->get_current_motor_state()->position;;
        // jointData_[i].vel_ = rb_->Motors[i]->get_current_motor_state()->velocity;
        jointData_[i].vel_ = motor_dq(i);
        jointData_[i].tau_ = motor_tau(i);
    }
    
    // realJointPosPublisher_.publish(createFloat64MultiArrayFromVector(motor_q));
    // realJointTauPublisher_.publish(createFloat64MultiArrayFromVector(motor_tau));

    // 设置默认指令 
    std::vector<std::string> names = hybridJointInterface_.getNames();
    for (const auto& name : names) {
        HybridJointHandle handle = hybridJointInterface_.getHandle(name);
        handle.setFeedforward(0.);
        handle.setVelocityDesired(0.);
        handle.setKd(0.5);
        // ROS_WARN("%s: %.3f", name.c_str(), handle.getPosition());
    }

    // rviz 可视化
    robotStatePublisherPtr_->publishFixedTransforms(true);
    tf::Transform baseTransform;
    baseTransform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    baseTransform.setRotation(tf::Quaternion(imuData_.ori_[0], imuData_.ori_[1], imuData_.ori_[2], imuData_.ori_[3]));
    // std::cerr << imuData_.ori_[0] << " " << imuData_.ori_[1] << " " << imuData_.ori_[2] << " " << imuData_.ori_[3] << std::endl;
    // baseTransform.setRotation(tf::Quaternion(0,0,0,1));
    tfBroadcaster_.sendTransform(tf::StampedTransform(baseTransform, time, "world", "base_footprint"));

    std::map<std::string, double> jointPositions{
        {"l_ankle_roll_joint", motor_q(0)}, {"l_ankle_pitch_joint",motor_q(1)}, {"l_knee_pitch_joint",motor_q(2)},
        {"l_hip_yaw_joint",motor_q(3)}, {"l_hip_roll_joint",motor_q(4)}, {"l_hip_pitch_joint",motor_q(5)},
        {"r_ankle_roll_joint",motor_q(6)}, {"r_ankle_pitch_joint",motor_q(7)}, {"r_knee_pitch_joint",motor_q(8)},
        {"r_hip_yaw_joint",motor_q(9)}, {"r_hip_roll_joint",motor_q(10)}, {"r_hip_pitch_joint",motor_q(11)}
    };
    robotStatePublisherPtr_->publishTransforms(jointPositions, time);
}

void RobotPaiHW::write(const ros::Time& time, const ros::Duration& period) {
    // joint -> motor
    VectorXd motor_des_q(12);
    for (int i = 0; i < 12; i++) {
        motor_des_q(i) = (jointData_[i].posDes_ - jointBias_[i]) * jointAxis_[i];
    }

    VectorXd kp(12), kd(12);
    kp << 0, 2.5, 0, 0, 0, 0, 0, 2.5, 0, 0, 0, 0;
    kp = kp;
    kd = kp/40;
    for (int i = 0; i < 12; i++) {
        rb_->Motors[i]->fresh_cmd_int16(motor_des_q(i),
                                        jointData_[i].velDes_,
                                        jointData_[i].ff_,
                                        jointData_[i].kp_,
                                        // kp(i),
                                        0.,
                                        jointData_[i].kd_,
                                        0., 0., 0.);
        // rb_->Motors[i]->fresh_cmd_int16(0,
        //                                 0,
        //                                 0,
        //                                 // jointData_[i].kp_,
        //                                 0,
        //                                 0.,
        //                                 0,
        //                                 0., 0., 0.);
        // if (test_torque) {
        //     if (i == 2) 
        //     rb_->Motors[i]->fresh_cmd_int16(0,
        //                                     0,
        //                                     1.,
        //                                     0,
        //                                     0.,
        //                                     0.,
        //                                     0., 0., 0.);
        // }
    }
    VectorXd desire_tau(12); desire_tau.setZero();
    // desire_tau(2) = 2.5 * (0 - jointData_[2].pos_) + 0.0625 * (0 - jointData_[2].vel_ );
    if (test_torque) desire_tau(2) = 1.0;
    // ROS_WARN_STREAM("left knee: pos: " << jointData_[2].pos_ << " tau: " << jointData_->tau_ << " des tau: " << desire_tau(2) << std::endl);
    rb_->motor_send_2();
    // rlTauPublisher_.publish(createFloat64MultiArrayFromVector(desire_tau));
}

bool RobotPaiHW::setupJoints() {
    std::vector<std::string> jointNames{"l_ankle_roll_joint", "l_ankle_pitch_joint", "l_knee_pitch_joint",
                                        "l_hip_yaw_joint", "l_hip_roll_joint", "l_hip_pitch_joint",
                                        "r_ankle_roll_joint", "r_ankle_pitch_joint", "r_knee_pitch_joint",
                                        "r_hip_yaw_joint", "r_hip_roll_joint", "r_hip_pitch_joint",};

    // std::vector<std::string> jointNames{"left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
    //                                     "left_knee_pitch_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    //                                     "right_hip_yaw_joint", "right_hip_roll_joint", "right_hip_pitch_joint",
    //                                     "right_knee_pitch_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"};
    
    for (int i = 0; i < jointNames.size(); i++) {
        hardware_interface::JointStateHandle state_handle(jointNames[i], &jointData_[i].pos_, &jointData_[i].vel_, &jointData_[i].tau_);
        jointStateInterface_.registerHandle(state_handle);
        hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[i].posDes_, &jointData_[i].velDes_,
                                                           &jointData_[i].kp_, &jointData_[i].kd_, &jointData_[i].ff_));
    }

    return true;
}

bool RobotPaiHW::setupImu() {
    imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("base_imu", "base_imu", imuData_.ori_, imuData_.oriCov_,
                                                                         imuData_.angularVel_, imuData_.angularVelCov_, imuData_.linearAcc_,
                                                                         imuData_.linearAccCov_));
    imuData_.oriCov_[0] = 0.0012;
    imuData_.oriCov_[4] = 0.0012;
    imuData_.oriCov_[8] = 0.0012;

    imuData_.angularVelCov_[0] = 0.0004;
    imuData_.angularVelCov_[4] = 0.0004;
    imuData_.angularVelCov_[8] = 0.0004;

    return true;
}

void RobotPaiHW::imuCallBack(const sensor_msgs::ImuConstPtr& msg) {
    imuData_.ori_[0] = msg->orientation.x;
    imuData_.ori_[1] = msg->orientation.y;
    imuData_.ori_[2] = msg->orientation.z;
    imuData_.ori_[3] = msg->orientation.w;
    imuData_.angularVel_[0] = msg->angular_velocity.x;
    imuData_.angularVel_[1] = msg->angular_velocity.y;
    imuData_.angularVel_[2] = msg->angular_velocity.z;
    imuData_.linearAcc_[0] = msg->linear_acceleration.x;
    imuData_.linearAcc_[1] = msg->linear_acceleration.y;
    imuData_.linearAcc_[2] = msg->linear_acceleration.z;
}

}