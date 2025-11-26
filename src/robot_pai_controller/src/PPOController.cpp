#include "robot_pai_controller/PPOController.h"
#include <pluginlib/class_list_macros.hpp>
#include "robot_pai_controller/RotationTools.h"

namespace robot_pai {

void PPOController::handleWalkMode() {
    if (loopCount_ % robotCfg_.controlCfg.decimation == 0) {
        double t = ros::Time::now().toSec();
        computeObservation();
        computeActions();
        ROS_WARN_STREAM("NEED TIME: " << ros::Time::now().toSec()-t << " s.");
        // limit action range
        scalar_t actionMin = -robotCfg_.clipActions;
        scalar_t actionMax = robotCfg_.clipActions;
        std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                    [actionMin, actionMax](scalar_t x) { return std::max(actionMin, std::min(actionMax, x)); });
    }
    // set action
    vector_t posdes(12), taudes(12);
    for (int i = 0; i < hybridJointHandles_.size(); i++) {
        scalar_t pos_des;
        pos_des = actions_[i] * robotCfg_.controlCfg.actionScale + defaultJointAngles_(i);
        // if (i == 1) pos_des = actions_[i] * robotCfg_.controlCfg.actionScale * 0.9 + defaultJointAngles_(i);
        posdes(i) = pos_des;
        taudes(i) = robotCfg_.controlCfg.stiffness[i] * (pos_des - propri_.jointPos(i)) + robotCfg_.controlCfg.damping[i] * (0 - propri_.jointVel(i));
        hybridJointHandles_[i].setCommand(pos_des, 0, robotCfg_.controlCfg.stiffness[i], robotCfg_.controlCfg.damping[i], 0);
        lastActions_(i, 0) = actions_[i];
    }
    // std::cerr << lastActions_.transpose() << std::endl;
    
    // realTorquePublisher_.publish(createFloat64MultiArrayFromVector(taudes));
    // rlJointPosPublisher_.publish(createFloat64MultiArrayFromVector(posdes));
}

bool PPOController::loadModel(ros::NodeHandle& nh) {
    std::string policyFilePath;
    if (!nh.getParam("/policyFile", policyFilePath)) {
        ROS_ERROR_STREAM("Get policy path fail from param server, some error occur!");
        return false;
    }
    policyFilePath_ = policyFilePath;
    ROS_INFO_STREAM("Load Onnx model from path : " << policyFilePath);

    // create env
    onnxEnvPrt_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController"));
    // create session
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetInterOpNumThreads(4);
    sessionPtr_ = std::make_unique<Ort::Session>(*onnxEnvPrt_, policyFilePath.c_str(), sessionOptions);
    // get input and output info
    inputNames_.clear();
    outputNames_.clear();
    inputShapes_.clear();
    outputShapes_.clear();
    Ort::AllocatorWithDefaultOptions allocator;
    ROS_INFO_STREAM("count: " << sessionPtr_->GetOutputCount());
    for (int i = 0; i < sessionPtr_->GetInputCount(); i++) {
        auto inputnamePtr = sessionPtr_->GetInputNameAllocated(i, allocator);
        inputNodeNameAllocatedStrings.push_back(std::move(inputnamePtr));
        inputNames_.push_back(inputNodeNameAllocatedStrings.back().get());
        // inputNames_.push_back(sessionPtr_->GetInputNameAllocated(i, allocator).get());
        inputShapes_.push_back(sessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
        std::cout << "Input Name [" << i << "]: " << inputnamePtr.get() << std::endl;
    }
    for (int i = 0; i < sessionPtr_->GetOutputCount(); i++) {
        auto outputnamePtr = sessionPtr_->GetOutputNameAllocated(i, allocator);
        outputNodeNameAllocatedStrings.push_back(std::move(outputnamePtr));
        outputNames_.push_back(outputNodeNameAllocatedStrings.back().get());
        // outputNames_.push_back(sessionPtr_->GetOutputNameAllocated(i, allocator).get());
        std::cout << sessionPtr_->GetOutputNameAllocated(i, allocator).get() << std::endl;
        outputShapes_.push_back(sessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
        std::cout << "Output Name [" << i << "]: " << outputnamePtr.get() << std::endl;
    }

    // 打印 inputNames_
    std::cout << "Input Names:" << std::endl;
    for (const char* name : inputNames_) {
        std::cout << name << std::endl;
    }

    // 打印 outputNames_
    std::cout << "Output Names:" << std::endl;
    for (const char* name : outputNames_) {
        std::cout << name << std::endl;
    }

    ROS_INFO_STREAM("Load Onnx model successfully !!!");
    return true;
}

bool PPOController::loadRLCfg(ros::NodeHandle& nh) {
    RobotCfg::InitState& initState = robotCfg_.initState;
    RobotCfg::ControlCfg& controlCfg = robotCfg_.controlCfg;
    RobotCfg::ObsScales& obsScales = robotCfg_.obsScales;

    int error = 0;
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/l_hip_yaw_joint", initState.l_hip_yaw_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/l_hip_roll_joint", initState.l_hip_roll_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/l_hip_pitch_joint", initState.l_hip_pitch_joint));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/l_knee_pitch_joint", initState.l_knee_pitch_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/l_ankle_pitch_joint", initState.l_ankle_pitch_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/l_ankle_roll_joint", initState.l_ankle_roll_joint));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/r_hip_yaw_joint", initState.r_hip_yaw_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/r_hip_roll_joint", initState.r_hip_roll_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/r_hip_pitch_joint", initState.r_hip_pitch_joint));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/r_knee_pitch_joint", initState.r_knee_pitch_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/r_ankle_pitch_joint", initState.r_ankle_pitch_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/r_ankle_roll_joint", initState.r_ankle_roll_joint));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness/hip_yaw_joint", controlCfg.stiffness_hip_yaw));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness/hip_roll_joint", controlCfg.stiffness_hip_roll));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness/hip_pitch_joint", controlCfg.stiffness_hip_pitch));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness/knee_pitch_joint", controlCfg.stiffness_knee_pitch));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness/ankle_pitch_joint", controlCfg.stiffness_ankle_pitch));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness/ankle_roll_joint", controlCfg.stiffness_ankle_roll));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping/hip_yaw_joint", controlCfg.damping_hip_yaw));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping/hip_roll_joint", controlCfg.damping_hip_roll));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping/hip_pitch_joint", controlCfg.damping_hip_pitch));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping/knee_pitch_joint", controlCfg.damping_knee_pitch));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping/ankle_pitch_joint", controlCfg.damping_ankle_pitch));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping/ankle_roll_joint", controlCfg.damping_ankle_roll));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/action_scale", controlCfg.actionScale));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/decimation", controlCfg.decimation));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/clip_scales/clip_observations", robotCfg_.clipObs));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/clip_scales/clip_actions", robotCfg_.clipActions));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/lin_vel", obsScales.linVel));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/ang_vel", obsScales.angVel));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/dof_pos", obsScales.dofPos));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/dof_vel", obsScales.dofVel));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/height_measurements", obsScales.heightMeasurements));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/quat", obsScales.quat));
    

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/actions_size", actionsSize_));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/observations_size", observationSize_));
    // error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/cmdTopicName", robotCfg_.cmdTopicName));

    actions_.resize(actionsSize_);  
    std::fill(actions_.begin(), actions_.end(), 0.0);
    observations_.resize(observationSize_);
    std::fill(observations_.begin(), observations_.end(), 0.0);

    controlCfg.stiffness.resize(12);
    controlCfg.damping.resize(12);
    controlCfg.stiffness << controlCfg.stiffness_hip_pitch, controlCfg.stiffness_hip_roll, controlCfg.stiffness_hip_yaw,
                            controlCfg.stiffness_knee_pitch, controlCfg.stiffness_ankle_pitch, controlCfg.stiffness_ankle_roll,
                            controlCfg.stiffness_hip_pitch, controlCfg.stiffness_hip_roll, controlCfg.stiffness_hip_yaw,
                            controlCfg.stiffness_knee_pitch, controlCfg.stiffness_ankle_pitch, controlCfg.stiffness_ankle_roll;
    
    controlCfg.damping << controlCfg.damping_hip_pitch, controlCfg.damping_hip_roll, controlCfg.damping_hip_yaw,
                          controlCfg.damping_knee_pitch, controlCfg.damping_ankle_pitch, controlCfg.damping_ankle_roll,
                          controlCfg.damping_hip_pitch, controlCfg.damping_hip_roll, controlCfg.damping_hip_yaw,
                          controlCfg.damping_knee_pitch, controlCfg.damping_ankle_pitch, controlCfg.damping_ankle_roll;
    command_.x = 0.0;
    command_.y = 0.0;
    command_.yaw = 0;

    baseLinVel_.setZero();
    basePosition_.setZero();
    std::vector<scalar_t> defaultJointAngles{
        robotCfg_.initState.l_hip_pitch_joint, robotCfg_.initState.l_hip_roll_joint, robotCfg_.initState.l_hip_yaw_joint,
        robotCfg_.initState.l_knee_pitch_joint, robotCfg_.initState.l_ankle_pitch_joint, robotCfg_.initState.l_ankle_roll_joint,
        robotCfg_.initState.r_hip_pitch_joint, robotCfg_.initState.r_hip_roll_joint, robotCfg_.initState.r_hip_yaw_joint,
        robotCfg_.initState.r_knee_pitch_joint, robotCfg_.initState.r_ankle_pitch_joint, robotCfg_.initState.r_ankle_roll_joint};

    lastActions_.resize(actuatedDofNum_);
    defaultJointAngles_.resize(actuatedDofNum_);
    for (int i = 0; i < actuatedDofNum_; i++) {
        defaultJointAngles_(i) = defaultJointAngles[i];
    }

    return (error == 0);
}

void PPOController::computeActions() {
    // 打印 inputNames_
    std::cout << "Input Names:" << std::endl;
    for (const char* name : inputNames_) {
        std::cout << name << std::endl;
    }

    // 打印 outputNames_
    std::cout << "Output Names:" << std::endl;
    for (const char* name : outputNames_) {
        std::cout << name << std::endl;
    }

    // create input tensor object
    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
    std::vector<Ort::Value> inputValues;
    inputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, observations_.data(), observations_.size(),
                                                                    inputShapes_[0].data(), inputShapes_[0].size()));
    // run inference
    Ort::RunOptions runOptions;
    std::vector<Ort::Value> outputValues = sessionPtr_->Run(runOptions, inputNames_.data(), inputValues.data(), 1, outputNames_.data(), 1);

    for (int i = 0; i < actionsSize_; i++) {
        actions_[i] = *(outputValues[0].GetTensorMutableData<tensor_element_t>() + i);
    }
}

void PPOController::computeObservation() {

    // command
    vector3_t command;

    // command[0] = command_.x * (command_.x > 0.05 || command_.x < -0.05);
    // command[1] = command_.y * (command_.y > 0.05 || command_.y < -0.05);
    // command[2] = command_.yaw * (command_.yaw > 0.1 || command_.yaw < -0.1);

    command[0] = command_.x;
    command[1] = command_.y;
    command[2] = command_.yaw;

    // actions
    vector_t actions(lastActions_);

    RobotCfg::ObsScales& obsScales = robotCfg_.obsScales;
    matrix_t commandScaler = Eigen::DiagonalMatrix<scalar_t, 3>(obsScales.linVel, obsScales.linVel, obsScales.angVel);

    // TODO
    // vector_t obs(47);
    vector_t obs(45);
    double phase, sin_pos, cos_pos;
    phase = t_ / 0.7;
    // if (command[0] == 0 && command[1] == 0 && command[2] == 0) 
    //     phase = 0.;
    sin_pos = sin(2 * M_PI * phase);
    cos_pos = cos(2 * M_PI * phase);
    // clang-format off
    // obs << sin_pos, cos_pos,
    //        commandScaler * command,
    //        (propri_.jointPos - defaultJointAngles_) * obsScales.dofPos,
    //         propri_.jointVel * obsScales.dofVel,
    //         actions,
    //         propri_.baseAngVel * obsScales.angVel,
    //         propri_.euler_xyz * obsScales.quat;

    obs << propri_.baseAngVel * obsScales.angVel ,
           propri_.projectedGravity,
           commandScaler * command,
           (propri_.jointPos - defaultJointAngles_) * obsScales.dofPos,
           propri_.jointVel * obsScales.dofVel,
           actions;
    // std::cerr << "*********************************************" << std::endl;
    // std::cerr << "sin cos:  " << sin_pos << " " << cos_pos << std::endl;
    // std::cerr << "command  " << command.transpose() << std::endl;
    // std::cerr << "joint_pos:  " << (propri_.jointPos - defaultJointAngles_).transpose() << std::endl;
    // std::cerr << "jointVel:  " << propri_.jointVel.transpose() << std::endl;
    // std::cerr << "actions:  " << actions.transpose() << std::endl;
    // std::cerr << "baseAngVel:  " << propri_.baseAngVel.transpose() << std::endl;
    // std::cerr << "euler_xyz:  " << propri_.euler_xyz.transpose() << std::endl;
    // obs.setZero();
    // ROS_WARN("xyz: %.3f, %.3f, %.3f", propri_.euler_xyz(0), propri_.euler_xyz(1), propri_.euler_xyz(2));

    // clang-format on
    // std::move(observations_.begin() + 47, observations_.end(), observations_.begin());
    // for (size_t i = 0; i < obs.size(); i++) {
    //     *(observations_.end()-47+i) = static_cast<tensor_element_t>(obs(i));
    // }

    std::move(observations_.begin() + 45, observations_.end(), observations_.begin());
    for (size_t i = 0; i < obs.size(); i++) {
        *(observations_.end()-45+i) = static_cast<tensor_element_t>(obs(i));
    }
    // std::fill(observations_.begin(), observations_.end(), 0.0);
    // Limit observation range
    scalar_t obsMin = -robotCfg_.clipObs;
    scalar_t obsMax = robotCfg_.clipObs;
    std::transform(observations_.begin(), observations_.end(), observations_.begin(),
                    [obsMin, obsMax](scalar_t x) { return std::max(obsMin, std::min(obsMax, x)); });

}

}

PLUGINLIB_EXPORT_CLASS(robot_pai::PPOController, controller_interface::ControllerBase)
