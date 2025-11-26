#include "robot_pai_hw/RobotPaiHW.h"
#include "robot_pai_hw/RobotPaiHWLoop.h"

int main (int argc, char* argv[]) {
    ros::init(argc, argv, "robot_pai_hw");
    ros::NodeHandle nh;                 // 全局
    ros::NodeHandle robotHwNh("~");     // 私人

    ros::AsyncSpinner spinner(3);
    spinner.start();

    try {
        // Create the hardware interface specific to your robot
        // ROS_WARN("123");
        std::shared_ptr<robot_pai::RobotPaiHW> robotPaiHW = std::make_shared<robot_pai::RobotPaiHW>();
        // Initialize the hardware interface:
        // 1. retrieve configuration from rosparam
        // 2. initialize the hardware and interface it with ros_control
        robotPaiHW->init(nh, robotHwNh);

        // Start the control loop
        robot_pai::RobotPaiHWLoop controlLoop(nh, robotPaiHW);
        
        // Wait until shutdown signal received
        ros::waitForShutdown();
    } catch (const ros::Exception& e) {
        ROS_FATAL_STREAM("Error in the hardware interface:\n"
                        << "\t" << e.what());
        return 1;
    }

    return 0;
}
