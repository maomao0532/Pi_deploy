#ifndef _CANBOARD_H_
#define _CANBOARD_H_
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include "robot_pai_hw/hardware/motor/canport.h"
class canboard
{
private:
    int CANport_num;
    ros::NodeHandle n;
    std::vector<canport*> CANport;
    // std::vector<motor> motor;
    // std::vector<std::shared_ptr<canport>> CANport;

public:
    canboard(int _CANboard_ID, std::vector<lively_serial *> *ser)
    {
        if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_ID) + "_CANboard/CANport_num", CANport_num))
        {
            // ROS_INFO("Got params CANport_num: %d",CANport_num);
        }
        else
        {
            ROS_ERROR("Faile to get params CANport_num");
        }
        for (size_t j = 1; j <= CANport_num; j++) // 一个串口对应一个CANport
        {
            CANport.push_back(new canport(j, _CANboard_ID, (*ser)[(_CANboard_ID - 1) * CANport_num + j - 1]));
        }
    }
    ~canboard() {}
    int get_CANport_num()
    {
        return CANport_num;
    }
    void push_CANport(std::vector<canport*> *_CANport)
    {
        for (canport *c : CANport)
        {
            _CANport->push_back(c);
        }
    }
    void motor_send()
    {
        for (canport *c : CANport)
        {
            c->motor_send();
        }
    }
    void motor_send_2()
    {
        for (canport *c : CANport)
        {
            c->motor_send_2();
        }
    }
    void set_stop()
    {
        for (canport *c : CANport)
        {
            c->set_stop();
        }
    }

    void set_reset()
    {
        for (canport *c : CANport)
        {
            c->set_reset();
        }
    }
    
    void set_port_motor_num()
    {
        for (canport *c : CANport)
        {
            c->set_motor_num();
        }
    }
    void send_get_motor_state_cmd()
    {
        for (canport *c : CANport)
        {
            c->send_get_motor_state_cmd();
        }
    }
    void set_reset_zero()
    {
        for (canport *c : CANport)
        {
            // if (c->set_conf_load() != 0)
            // {
            //     return;
            // }
            for (int i = 0; i < 5; i++)
            {
                c->set_reset();
                c->motor_send_2();
                ros::Duration(0.1).sleep();
            }
            ros::Duration(1).sleep();
            if (c->set_reset_zero() == 0)
            {
                c->set_conf_write();
            }
            c->set_reset();
            c->motor_send_2();
            ros::Duration(1).sleep();
            c->motor_send_2();
            ros::Duration(1).sleep();
        }
    }
    void set_motor_runzero()
    {
        for (canport *c : CANport)
        {
            c->set_motor_runzero();
        }
    }
};
#endif
