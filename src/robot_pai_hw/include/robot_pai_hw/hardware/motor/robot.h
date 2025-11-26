#ifndef _ROBOT_H_
#define _ROBOT_H_
#include <iostream>
#include <ros/ros.h>
#include <thread>
#include <initializer_list>
#include <libserialport.h>
#include <fstream>
#include <dirent.h>
#include "robot_pai_hw/hardware/motor/canboard.h"

// #define DYNAMIC_CONFIG_ROBOT
#ifdef DYNAMIC_CONFIG_ROBOT
#include <dynamic_reconfigure/server.h>
#include <livelybot_serial/robot_dynamic_config_20Config.h>
#include <libserialport.h>
#include <dirent.h>
#include <algorithm>
#include <sensor_msgs/JointState.h>
#endif
namespace livelybot_serial
{
    class robot
    {
    private:
        std::string robot_name, Serial_Type, CAN_type, CANboard_type, Serial_allocate;
        int arm_dof, leg_dof, CANboard_num, Seial_baudrate, SDK_version;
        ros::NodeHandle n;
        std::vector<canboard> CANboards;
        std::vector<std::string> str;
        std::vector<lively_serial *> ser;
        float SDK_version2 = 3.5; // SDK版本
#ifdef DYNAMIC_CONFIG_ROBOT
        std::vector<double> config_slope_posistion;
        std::vector<double> config_offset_posistion;
        std::vector<double> config_slope_torque;
        std::vector<double> config_offset_torque;
        std::vector<double> config_slope_velocity;
        std::vector<double> config_offset_velocity;
        std::vector<double> config_rkp;
        std::vector<double> config_rkd;
        dynamic_reconfigure::Server<livelybot_serial::robot_dynamic_config_20Config> dr_srv_;

#endif
    public:
        std::vector<motor *> Motors;
        // std::vector<std::shared_ptr<canport>> CANPorts;
        std::vector<canport *> CANPorts;
        std::vector<std::thread> ser_recv_threads, send_threads;
        // std::thread pos_protect;
        int motor_position_limit_flag = 0;
        int motor_torque_limit_flag = 0;
        robot()
        {
            if (n.getParam("robot/SDK_version", SDK_version))
            {
                // ROS_INFO("Got params SDK_version: %f",SDK_version);
            }
            else
            {
                ROS_ERROR("Faile to get params SDK_version");
                SDK_version = -1;
            }
            if (n.getParam("robot/Seial_baudrate", Seial_baudrate))
            {
                // ROS_INFO("Got params seial_baudrate: %s",seial_baudrate.c_str());
            }
            else
            {
                ROS_ERROR("Faile to get params seial_baudrate");
            }
            if (n.getParam("robot/robot_name", robot_name))
            {
                // ROS_INFO("Got params robot_name: %s",robot_name.c_str());
            }
            else
            {
                ROS_ERROR("Faile to get params robot_name");
            }
            if (n.getParam("robot/CANboard_num", CANboard_num))
            {
                // ROS_INFO("Got params CANboard_num: %d",CANboard_num);
            }
            else
            {
                ROS_ERROR("Faile to get params CANboard_num");
            }
            if (n.getParam("robot/CANboard_type", CANboard_type))
            {
                // ROS_INFO("Got params CANboard_type: %s",CANboard_type.c_str());
            }
            else
            {
                ROS_ERROR("Faile to get params CANboard_type");
            }
            if (n.getParam("robot/CANboard_type", CANboard_type))
            {
                // ROS_INFO("Got params CANboard_type: %s",CANboard_type.c_str());
            }
            else
            {
                ROS_ERROR("Faile to get params CANboard_type");
            }
            if (n.getParam("robot/Serial_Type", Serial_Type))
            {
                // ROS_INFO("Got params Serial_Type: %s",Serial_Type.c_str());
            }
            else
            {
                ROS_ERROR("Faile to get params Serial_Type");
            }
            if (n.getParam("robot/Serial_allocate", Serial_allocate))
            {
                // ROS_INFO("Got params Serial_Type: %s",Serial_Type.c_str());
            }
            else
            {
                ROS_ERROR("Faile to get params Serial_allocate");
            }

            ROS_INFO("\033[1;32mGot params SDK_version: %.1fv\033[0m", SDK_version2);
            ROS_INFO("\033[1;32mThe robot name is %s\033[0m", robot_name.c_str());
            ROS_INFO("\033[1;32mThe robot has %d CANboards\033[0m", CANboard_num);
            ROS_INFO("\033[1;32mThe CANboard type is %s\033[0m", CANboard_type.c_str());
            ROS_INFO("\033[1;32mThe Serial type is %s\033[0m", Serial_Type.c_str());
            ROS_INFO("\033[1;32mThe Serial allocate type is %s\033[0m", Serial_allocate.c_str());
            init_ser();
            // if (Serial_allocate == "1for2")
            {
                for (size_t i = 1; i <= CANboard_num; i++) // 一个CANboard使用两个串口
                {
                    CANboards.push_back(canboard(i, &ser));
                }
            }

            for (canboard &cb : CANboards)
            {
                cb.push_CANport(&CANPorts);
            }
            for (canport *cp : CANPorts)
            {
                // std::thread(&canport::send, &cp);
                cp->puch_motor(&Motors);
            }
            set_port_motor_num(); // 设置通道上挂载的电机数，并获取主控板固件版本号
            chevk_motor_connection();  // 检测电机连接是否正常

            ros::Duration(0.1).sleep();

            ROS_INFO("\033[1;32mThe robot has %ld motors\033[0m", Motors.size());
            ROS_INFO("robot init");
            // for (motor m:Motors)
            // {
            //     std::cout<<m.get_motor_belong_canboard()<<" "<<m.get_motor_belong_canport()<<" "<<m.get_motor_id()<<std::endl;
            // }

#ifdef DYNAMIC_CONFIG_ROBOT
            config_slope_posistion = std::vector<double>(20, 1);
            config_offset_posistion = std::vector<double>(20, 0);
            config_slope_torque = std::vector<double>(20, 1);
            config_offset_torque = std::vector<double>(20, 0);
            config_slope_velocity = std::vector<double>(20, 1);
            config_offset_velocity = std::vector<double>(20, 0);
            config_rkp = std::vector<double>(20, 3);
            config_rkd = std::vector<double>(20, 0.01);
            dynamic_reconfigure::Server<livelybot_serial::robot_dynamic_config_20Config>::CallbackType cb;
            cb = boost::bind(&robot::configCallback, this, _1, _2);
            dr_srv_.setCallback(cb);
#endif
        }
        ~robot()
        {
            set_stop();
            motor_send_2();
            for (auto &thread : ser_recv_threads)
            {
                if (thread.joinable())
                    thread.join();
            }
        }

        void detect_motor_limit()
        {
            // 电机正常运行时检测是否超过限位，停机之后不检测
            if(!motor_position_limit_flag && !motor_torque_limit_flag)
            {
                for (motor *m : Motors)
                {
                    if(m->pos_limit_flag)
                    {                    
                        ROS_ERROR("robot pos limit, motor stop.");
                        set_stop();
                        motor_position_limit_flag = m->pos_limit_flag;
                        break;
                    }

                    if(m->tor_limit_flag)
                    {
                        ROS_ERROR("robot torque limit, motor stop.");
                        set_stop();
                        motor_torque_limit_flag = m->tor_limit_flag;
                        break;
                    }
                }            
            }
        }

        void motor_send()
        {
            for (canboard &cb : CANboards)
            {
                cb.motor_send();
                // ROS_INFO("ok");
            }
        }
        void motor_send_2()
        {
            if(!motor_position_limit_flag && !motor_torque_limit_flag)
            {
                for (canboard &cb : CANboards)
                {
                    cb.motor_send_2();
                }
            }
            
        }

        int serial_pid_vid(const char *name, int *pid, int *vid)
        {
            int r = 0;
            struct sp_port *port;
            
            sp_get_port_by_name(name, &port);
            sp_open(port, SP_MODE_READ);
            if (sp_get_port_usb_vid_pid(port, vid, pid) != SP_OK) 
            {
                r = 1;
            } 
            std::cout << "Port: " << name << ", PID: 0x" << std::hex << *pid << ", VID: 0x" << *vid << std::dec << std::endl;

            // 关闭端口
            sp_close(port);
            sp_free_port(port);

            return r;
        }

        int serial_pid_vid(const char *name)
        {
            int pid, vid;
            int r = 0;
            struct sp_port *port;
            
            sp_get_port_by_name(name, &port);
            sp_open(port, SP_MODE_READ);
            if (sp_get_port_usb_vid_pid(port, &vid, &pid) != SP_OK) 
            {
                r = -1;
            } 
            else 
            {
                switch (vid)
                {
                case (0xCAF1):
                    r = 1;
                    break;
                case (0xCAF2):
                    r = 2;
                    break;
                default:
                    r = -2;
                    break;
                }
            }
            // std::cout << "Port: " << name << ", PID: 0x" << std::hex << pid << ", VID: 0x" << vid << std::dec << std::endl;

            // 关闭端口
            sp_close(port);
            sp_free_port(port);

            return r;
        }

        std::vector<std::string> list_serial_ports(const std::string& full_prefix) 
        {
            std::string base_path = full_prefix.substr(0, full_prefix.rfind('/') + 1);
            std::string prefix = full_prefix.substr(full_prefix.rfind('/') + 1);
            std::vector<std::string> serial_ports;
            DIR *directory;
            struct dirent *entry;

            directory = opendir(base_path.c_str());
            if (!directory)
            {
                std::cerr << "Could not open the directory " << base_path << std::endl;
                return serial_ports; // Return an empty vector if cannot open directory
            }

            while ((entry = readdir(directory)) != NULL)
            {
                std::string entryName = entry->d_name;
                if (entryName.find(prefix) == 0)
                { // Check if the entry name starts with the given prefix
                    serial_ports.push_back(base_path + entryName);
                }
            }

            closedir(directory);

            // Sort the vector in ascending order
            std::sort(serial_ports.begin(), serial_ports.end());

            return serial_ports;
        }

        void init_ser()
        {
            std::vector<std::string> ports = list_serial_ports(Serial_Type);
            for (const std::string& port : ports) 
            {
                if (serial_pid_vid(port.c_str()) > 0)
                {
                    ROS_INFO("Serial Port%ld = %s", str.size(), port.c_str());
                    str.push_back(port);
                }
            }

            if ((str.size() < 4 * CANboard_num))
            {
                ROS_ERROR("Cannot find the motor serial port, please check if the USB connection is normal.");
                exit(-1);
            }

            if (CANboard_num > 1)
            {
                std::vector<std::string> str1, str2;
                for (size_t i = 0; i < 8; i++)
                {   
                    int vid = serial_pid_vid(str[i].c_str());
                    if (vid == 1)
                    {
                        str1.push_back(str[i]);
                    }
                    else if (vid == 2)
                    {
                        str2.push_back(str[i]);
                    }
                    else
                    {
                        ROS_ERROR("Failed to open serial port.");
                        exit(-1);
                    }
                }

                for (size_t i = 0; i < 4; i++)
                {
                    std::swap(str[i], str1[i]);
                    std::swap(str[i + 4], str2[i]);
                }
            }
            std::cout << std::endl;
            for (int i = 0; i < 4*CANboard_num; i++)
            {
                std::cout << str[i] << " " << serial_pid_vid(str[i].c_str()) << std::endl;
            }
            
            for (size_t i = 0; i < str.size(); i++)
            {
                lively_serial *s = new lively_serial(&str[i], Seial_baudrate, 1);
                ser.push_back(s);
                if (SDK_version == 2)
                {
                    /* code */
                    ser_recv_threads.push_back(std::thread(&lively_serial::recv_1for6_42, s));
                }
                else
                {
                    ROS_ERROR("SDK_version != 2");
                }
            }
        }
        void test_ser_motor()
        {
            for (lively_serial *s : ser)
            {
                s->test_ser_motor();
            }
        }

        /**
         * @brief 设置每个通道的电机数量，并查询主控板固件版本
         */
        void set_port_motor_num()
        {
            for (canboard &cb : CANboards)
            {
                cb.set_port_motor_num();
            }
        }
        void send_get_motor_state_cmd()
        {
            for (canboard &cb : CANboards)
            {
                cb.send_get_motor_state_cmd();
            }
        }

        void chevk_motor_connection()
        {
            int t = 0;
            int num = 0;
            std::vector<int> board;
            std::vector<int> port;
            std::vector<int> id;

#define MAX_DELAY 2000 // 单位 ms

            ROS_INFO("Detecting motor connection");
            while (t++ < MAX_DELAY)
            {
                send_get_motor_state_cmd();
                ros::Duration(0.001).sleep();

                num = 0;
                std::vector<int>().swap(board);
                std::vector<int>().swap(port);
                std::vector<int>().swap(id);
                for (motor *m : Motors)
                {
                    if (m->get_current_motor_state()->position != 999.0f)
                    {
                        ++num;
                    }
                    else
                    {
                        board.push_back(m->get_motor_belong_canboard());
                        port.push_back(m->get_motor_belong_canport());
                        id.push_back(m->get_motor_id());
                    }
                }

                if (num == Motors.size())
                {
                    break;
                }

                if (t % 1000 == 0)
                {
                    ROS_INFO(".");
                }
            }

            if (num == Motors.size())
            {
                ROS_INFO("\033[1;32mAll motor connections are normal\033[0m");
            }
            else
            {
                for (int i = 0; i < Motors.size() - num; i++)
                {
                    ROS_ERROR("CANboard(%d) CANport(%d) id(%d) Motor connection disconnected!!!", board[i], port[i], id[i]);
                }
                // exit(-1);
                ros::Duration(1).sleep();
            }
        }

        void set_stop()
        {
            for (canboard &cb : CANboards)
            {
                cb.set_stop();
            }
            motor_send_2();
        }

        void set_reset()
        {
            for (canboard &cb : CANboards)
            {
                cb.set_reset();
            }
        }

        void set_reset_zero()
        {
            for (canboard &cb : CANboards)
            {
                cb.set_reset_zero();
            }
        }

        void set_reset_zero(std::initializer_list<int> motors)
        {
            for (auto const &motor : motors)
            {
                int board_id = Motors[motor]->get_motor_belong_canboard() - 1;
                int port_id = Motors[motor]->get_motor_belong_canport() - 1;
                int motor_id = Motors[motor]->get_motor_id();
                ROS_INFO("%d, %d, %d\n", board_id, port_id, motor_id);

                if (CANPorts[port_id]->set_conf_load(motor_id) != 0)
                {
                    ROS_ERROR("Motor %d settings restoration failed.", motor);
                    return;
                }
                
                ROS_INFO("Motor %d settings have been successfully restored. Initiating zero position reset.", motor);
                if (CANPorts[port_id]->set_reset_zero(motor_id) == 0)
                {
                    ROS_INFO("Motor %d reset to zero position successfully, awaiting settings save.null", motor);
                    if (CANPorts[port_id]->set_conf_write(motor_id) == 0)
                    {
                        ROS_INFO("Motor %d settings saved successfully.", motor);
                    }
                    else
                    {
                        ROS_ERROR("Motor %d settings saved failed.", motor);
                    }
                }
                else
                {
                    ROS_ERROR("Motor %d reset to zero position failed.", motor);
                }
            }
        }

        void set_motor_runzero()
        {
            for (int i = 0; i < 5; i++)
            {
                for (canboard &cb : CANboards)
                {
                    cb.set_motor_runzero();
                }
                ros::Duration(0.01).sleep();
            }
            ros::Duration(4).sleep();
        }

#ifdef DYNAMIC_CONFIG_ROBOT
        void configCallback(robot_dynamic_config_20Config &config, uint32_t level)
        {
            ROS_INFO("reconfigure parameter");
            // 更新vector参数
            config_slope_posistion[0] = (config.position_slope_0);
            config_slope_posistion[1] = (config.position_slope_1);
            config_slope_posistion[2] = (config.position_slope_2);
            config_slope_posistion[3] = (config.position_slope_3);
            config_slope_posistion[4] = (config.position_slope_4);
            config_slope_posistion[5] = (config.position_slope_5);
            config_slope_posistion[6] = (config.position_slope_6);
            config_slope_posistion[7] = (config.position_slope_7);
            config_slope_posistion[8] = (config.position_slope_8);
            config_slope_posistion[9] = (config.position_slope_9);
            config_slope_posistion[10] = (config.position_slope_10);
            config_slope_posistion[11] = (config.position_slope_11);
            config_slope_posistion[12] = (config.position_slope_12);
            config_slope_posistion[13] = (config.position_slope_13);
            config_slope_posistion[14] = (config.position_slope_14);
            config_slope_posistion[15] = (config.position_slope_15);
            config_slope_posistion[16] = (config.position_slope_16);
            config_slope_posistion[17] = (config.position_slope_17);
            config_slope_posistion[18] = (config.position_slope_18);
            config_slope_posistion[19] = (config.position_slope_19);

            config_offset_posistion[0] = (config.position_offset_0);
            config_offset_posistion[1] = (config.position_offset_1);
            config_offset_posistion[2] = (config.position_offset_2);
            config_offset_posistion[3] = (config.position_offset_3);
            config_offset_posistion[4] = (config.position_offset_4);
            config_offset_posistion[5] = (config.position_offset_5);
            config_offset_posistion[6] = (config.position_offset_6);
            config_offset_posistion[7] = (config.position_offset_7);
            config_offset_posistion[8] = (config.position_offset_8);
            config_offset_posistion[9] = (config.position_offset_9);
            config_offset_posistion[10] = (config.position_offset_10);
            config_offset_posistion[11] = (config.position_offset_11);
            config_offset_posistion[12] = (config.position_offset_12);
            config_offset_posistion[13] = (config.position_offset_13);
            config_offset_posistion[14] = (config.position_offset_14);
            config_offset_posistion[15] = (config.position_offset_15);
            config_offset_posistion[16] = (config.position_offset_16);
            config_offset_posistion[17] = (config.position_offset_17);
            config_offset_posistion[18] = (config.position_offset_18);
            config_offset_posistion[19] = (config.position_offset_19);

            config_slope_velocity[0] = (config.velocity_slope_0);
            config_slope_velocity[1] = (config.velocity_slope_1);
            config_slope_velocity[2] = (config.velocity_slope_2);
            config_slope_velocity[3] = (config.velocity_slope_3);
            config_slope_velocity[4] = (config.velocity_slope_4);
            config_slope_velocity[5] = (config.velocity_slope_5);
            config_slope_velocity[6] = (config.velocity_slope_6);
            config_slope_velocity[7] = (config.velocity_slope_7);
            config_slope_velocity[8] = (config.velocity_slope_8);
            config_slope_velocity[9] = (config.velocity_slope_9);
            config_slope_velocity[10] = (config.velocity_slope_10);
            config_slope_velocity[11] = (config.velocity_slope_11);
            config_slope_velocity[12] = (config.velocity_slope_12);
            config_slope_velocity[13] = (config.velocity_slope_13);
            config_slope_velocity[14] = (config.velocity_slope_14);
            config_slope_velocity[15] = (config.velocity_slope_15);
            config_slope_velocity[16] = (config.velocity_slope_16);
            config_slope_velocity[17] = (config.velocity_slope_17);
            config_slope_velocity[18] = (config.velocity_slope_18);
            config_slope_velocity[19] = (config.velocity_slope_19);

            config_offset_velocity[0] = (config.velocity_offset_0);
            config_offset_velocity[1] = (config.velocity_offset_1);
            config_offset_velocity[2] = (config.velocity_offset_2);
            config_offset_velocity[3] = (config.velocity_offset_3);
            config_offset_velocity[4] = (config.velocity_offset_4);
            config_offset_velocity[5] = (config.velocity_offset_5);
            config_offset_velocity[6] = (config.velocity_offset_6);
            config_offset_velocity[7] = (config.velocity_offset_7);
            config_offset_velocity[8] = (config.velocity_offset_8);
            config_offset_velocity[9] = (config.velocity_offset_9);
            config_offset_velocity[10] = (config.velocity_offset_10);
            config_offset_velocity[11] = (config.velocity_offset_11);
            config_offset_velocity[12] = (config.velocity_offset_12);
            config_offset_velocity[13] = (config.velocity_offset_13);
            config_offset_velocity[14] = (config.velocity_offset_14);
            config_offset_velocity[15] = (config.velocity_offset_15);
            config_offset_velocity[16] = (config.velocity_offset_16);
            config_offset_velocity[17] = (config.velocity_offset_17);
            config_offset_velocity[18] = (config.velocity_offset_18);
            config_offset_velocity[19] = (config.velocity_offset_19);

            config_slope_torque[0] = (config.torque_slope_0);
            config_slope_torque[1] = (config.torque_slope_1);
            config_slope_torque[2] = (config.torque_slope_2);
            config_slope_torque[3] = (config.torque_slope_3);
            config_slope_torque[4] = (config.torque_slope_4);
            config_slope_torque[5] = (config.torque_slope_5);
            config_slope_torque[6] = (config.torque_slope_6);
            config_slope_torque[7] = (config.torque_slope_7);
            config_slope_torque[8] = (config.torque_slope_8);
            config_slope_torque[9] = (config.torque_slope_9);
            config_slope_torque[10] = (config.torque_slope_10);
            config_slope_torque[11] = (config.torque_slope_11);
            config_slope_torque[12] = (config.torque_slope_12);
            config_slope_torque[13] = (config.torque_slope_13);
            config_slope_torque[14] = (config.torque_slope_14);
            config_slope_torque[15] = (config.torque_slope_15);
            config_slope_torque[16] = (config.torque_slope_16);
            config_slope_torque[17] = (config.torque_slope_17);
            config_slope_torque[18] = (config.torque_slope_18);
            config_slope_torque[19] = (config.torque_slope_19);

            config_offset_torque[0] = (config.torque_offset_0);
            config_offset_torque[1] = (config.torque_offset_1);
            config_offset_torque[2] = (config.torque_offset_2);
            config_offset_torque[3] = (config.torque_offset_3);
            config_offset_torque[4] = (config.torque_offset_4);
            config_offset_torque[5] = (config.torque_offset_5);
            config_offset_torque[6] = (config.torque_offset_6);
            config_offset_torque[7] = (config.torque_offset_7);
            config_offset_torque[8] = (config.torque_offset_8);
            config_offset_torque[9] = (config.torque_offset_9);
            config_offset_torque[10] = (config.torque_offset_10);
            config_offset_torque[11] = (config.torque_offset_11);
            config_offset_torque[12] = (config.torque_offset_12);
            config_offset_torque[13] = (config.torque_offset_13);
            config_offset_torque[14] = (config.torque_offset_14);
            config_offset_torque[15] = (config.torque_offset_15);
            config_offset_torque[16] = (config.torque_offset_16);
            config_offset_torque[17] = (config.torque_offset_17);
            config_offset_torque[18] = (config.torque_offset_18);
            config_offset_torque[19] = (config.torque_offset_19);

            config_rkp[0] = config.rkp_0;
            config_rkp[1] = config.rkp_1;
            config_rkp[2] = config.rkp_2;
            config_rkp[3] = config.rkp_3;
            config_rkp[4] = config.rkp_4;
            config_rkp[5] = config.rkp_5;
            config_rkp[6] = config.rkp_6;
            config_rkp[7] = config.rkp_7;
            config_rkp[8] = config.rkp_8;
            config_rkp[9] = config.rkp_9;
            config_rkp[10] = config.rkp_10;
            config_rkp[11] = config.rkp_11;
            config_rkp[12] = config.rkp_12;
            config_rkp[13] = config.rkp_13;
            config_rkp[14] = config.rkp_14;
            config_rkp[15] = config.rkp_15;
            config_rkp[16] = config.rkp_16;
            config_rkp[17] = config.rkp_17;
            config_rkp[18] = config.rkp_18;
            config_rkp[19] = config.rkp_19;

            config_rkd[0] = config.rkd_0;
            config_rkd[1] = config.rkd_1;
            config_rkd[2] = config.rkd_2;
            config_rkd[3] = config.rkd_3;
            config_rkd[4] = config.rkd_4;
            config_rkd[5] = config.rkd_5;
            config_rkd[6] = config.rkd_6;
            config_rkd[7] = config.rkd_7;
            config_rkd[8] = config.rkd_8;
            config_rkd[9] = config.rkd_9;
            config_rkd[10] = config.rkd_10;
            config_rkd[11] = config.rkd_11;
            config_rkd[12] = config.rkd_12;
            config_rkd[13] = config.rkd_13;
            config_rkd[14] = config.rkd_14;
            config_rkd[15] = config.rkd_15;
            config_rkd[16] = config.rkd_16;
            config_rkd[17] = config.rkd_17;
            config_rkd[18] = config.rkd_18;
            config_rkd[19] = config.rkd_19;
        }

        void fresh_cmd_dynamic_config(float pos, float vel, float torque, size_t motor_idx)
        {
            if (motor_idx >= Motors.size())
            {
                ROS_ERROR("motor_idx greater than motors vector size!!!");
                return;
            }
            Motors[motor_idx]->fresh_cmd_int16((pos * config_slope_posistion[motor_idx]) + config_offset_posistion[motor_idx],
                                               (vel * config_slope_velocity[motor_idx]) + config_offset_velocity[motor_idx],
                                               (torque * config_slope_torque[motor_idx]) + config_offset_torque[motor_idx],
                                               config_rkp[motor_idx], 0, config_rkd[motor_idx], 0, 0, 0);
        }

        void fresh_cmd_dynamic_config(float pos, float vel, float torque, float kp, float kd, size_t motor_idx)
        {
            if (motor_idx >= Motors.size())
            {
                ROS_ERROR("motor_idx greater than motors vector size!!!");
                return;
            }
            Motors[motor_idx]->fresh_cmd_int16((pos * config_slope_posistion[motor_idx]) + config_offset_posistion[motor_idx],
                                               (vel * config_slope_velocity[motor_idx]) + config_offset_velocity[motor_idx],
                                               (torque * config_slope_torque[motor_idx]) + config_offset_torque[motor_idx],
                                               kp, 0, kd, 0, 0, 0);
        }

        void get_motor_state_dynamic_config(float &pos, float &vel, float &torque, size_t motor_idx)
        {
            if (motor_idx >= Motors.size())
            {
                ROS_ERROR("motor_idx greater than motors vector size!!!");
                return;
            }
            motor_back_t motor_back_data;
            motor_back_data = *Motors[motor_idx]->get_current_motor_state();
            pos = (motor_back_data.position - config_offset_posistion[motor_idx]) / config_slope_posistion[motor_idx];
            vel = (motor_back_data.velocity - config_offset_velocity[motor_idx]) / config_slope_velocity[motor_idx];
            torque = (motor_back_data.torque - config_offset_torque[motor_idx]) / config_slope_torque[motor_idx];
        }
#endif
    };
}
#endif
