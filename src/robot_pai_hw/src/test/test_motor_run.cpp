#include <ros/ros.h>
#include <iostream>
#include <thread>
#include <condition_variable>
#include "robot_pai_hw/hardware/serial_struct.h"
#include "robot_pai_hw/hardware/motor/robot.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_motor");
    ros::NodeHandle n;
    ros::Rate r(500);
    livelybot_serial::robot rb;
    ROS_WARN("its therse");
    //rb.set_motor_runzero();     // 电机上电自动回零
    ROS_INFO("\033[1;32mSTART\033[0m");
    // ========================== singlethread send =====================
    float derta = 0.01;//角度
    int cont = 0;
    float angle = 0.0;
    while (ros::ok()) // 此用法为逐个电机发送控制指令
    {
        /////////////////////////send
        int i = 0;
        for (motor *m : rb.Motors)
        {   
            // ROS_WARN("123");
            printf("id %d pos %f vel %f tqe %f\n", m->get_current_motor_state()->ID, m->get_current_motor_state()->position, m->get_current_motor_state()->velocity, m->get_current_motor_state()->torque);
            // printf("%4.2f ", m->get_current_motor_state()->position);
            m->fresh_cmd_int16(0, 0, 0, 0.0, 0, 0, 0.0, 0, 0.0);
            if (i == 8 || i == 9) {
                m->fresh_cmd_int16(0., 0., 0., 0.0, 0, 0, 0, 0, 0);
            }
            i++;
        }
        printf("\n");
        rb.motor_send_2();
        
        r.sleep();
    }

    ROS_INFO_STREAM("END"); 
    ros::spin();
    return 0;
}
