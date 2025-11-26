#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "robot_pai_hw/hardware/serial_struct.h"
#include <stdint.h>
#include "ros/ros.h"

#define my_2pi (6.28318530717f)
#define my_pi (3.14159265358f)

#define MEM_INDEX_ID(id) ((id) - 1)    

enum motor_type
{
    null,
    _5046,    // 黑色 圆形
    _4538,    // 银色 圆形
    _5047_36, // 5047 双级 36减速比 黑色 方形
    _5047_9,  // 5047 单级 9减速比 黑色 方形
    _4438_32, // 4438 双极 32减速比 黑色 方形
    _4438_8,  // 4438 单极 8减速比 黑色 方形
    _7136_7,  // 
};

enum pos_vel_convert_type
{
    radian_2pi = 0,  // 弧度制
    angle_360,       // 角度制
    turns,           // 圈数
};

class motor
{
private:
    int type, id, num, CANport_num, CANboard_num;//, iid;
    ros::NodeHandle n;
    motor_back_t data;
    std::string motor_name;
    motor_type type_ = motor_type::null;
    cdc_tr_message_s *p_cdc_tx_message = NULL;
    int id_max = 0;
    int control_type = 0;
    pos_vel_convert_type pos_vel_type = radian_2pi; 
    // 新增 限位判断
    bool pos_limit_enable = false; 
    float pos_upper = 0.0f;
    float pos_lower = 0.0f;
    bool tor_limit_enable = false;
    float tor_upper = 0.0f;
    float tor_lower = 0.0f;

public:
    motor_pos_val_tqe_rpd_s cmd_int16_5param;
    int pos_limit_flag = 0;     // 0 表示正常，1 表示超出上限， -1 表示超出下限
    int tor_limit_flag = 0;     // 0 表示正常，1 表示超出上限
    cdc_acm_rx_message_t cmd;
    motor(int _motor_num, int _CANport_num, int _CANboard_num, cdc_tr_message_s *_p_cdc_tx_message, int _id_max) : CANport_num(_CANport_num), CANboard_num(_CANboard_num), p_cdc_tx_message(_p_cdc_tx_message), id_max(_id_max)
    {
        if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/name", motor_name))
        {
            // ROS_INFO("Got params name: %s",motor_name);
        }
        else
        {
            ROS_ERROR("Faile to get params name");
        }
        if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/id", id))
        {
            // ROS_INFO("Got params id: %d",id);
        }
        else
        {
            ROS_ERROR("Faile to get params id");
        }
        if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/type", type))
        {
            // ROS_INFO("Got params type: %d",type);
        }
        else
        {
            ROS_ERROR("Faile to get params type");
        }
        if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/num", num))
        {
            // ROS_INFO("Got params num: %d",num);
        }
        else
        {
            ROS_ERROR("Faile to get params num");
        }
        // position limit
        if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/pos_limit_enable", pos_limit_enable))
        {
            ROS_INFO("Got params pos_limit_enable: %s",pos_limit_enable?"true":"false");
        }
        else
        {
            ROS_ERROR("Faile to get params pos_upper");
        }
        if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/pos_upper", pos_upper))
        {
            ROS_INFO("Got params pos_upper: %f",pos_upper);
        }
        else
        {
            ROS_ERROR("Faile to get params pos_upper");
        }
        if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/pos_lower", pos_lower))
        {
            ROS_INFO("Got params pos_lower: %f",pos_lower);
        }
        else
        {
            ROS_ERROR("Faile to get params pos_lower");
        }
        // torque limit
        if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/tor_limit_enable", tor_limit_enable))
        {
            ROS_INFO("Got params tor_limit_enable: %s",tor_limit_enable?"true":"false");
        }
        else
        {
            ROS_ERROR("Faile to get params tor_upper");
        }
        if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/tor_upper", tor_upper))
        {
            ROS_INFO("Got params tor_upper: %f",tor_upper);
        }
        else
        {
            ROS_ERROR("Faile to get params tor_upper");
        }
        if (n.getParam("robot/CANboard/No_" + std::to_string(_CANboard_num) + "_CANboard/CANport/CANport_" + std::to_string(_CANport_num) + "/motor/motor" + std::to_string(_motor_num) + "/tor_lower", tor_lower))
        {
            ROS_INFO("Got params tor_lower: %f",tor_lower);
        }
        else
        {
            ROS_ERROR("Faile to get params tor_lower");
        }
        if (n.getParam("robot/control_type", control_type))
        {
            // ROS_INFO("Got params ontrol_type: %f",SDK_version);
            
        }
        else
        {
            ROS_ERROR("Faile to get params control_type");
        }
        set_motor_type(type);
        memset(&cmd, 0, sizeof(cmd));
        memset(&data, 0, sizeof(data));
        cmd.motor_cmd.ID = id;
        cmd.head[0] = 0xFE;
        data.ID = id;
        // iid = id - 1;
        data.position = 999.0f;
    }
    ~motor() {}
    template <typename T>
    inline T float2int(float in_data, uint8_t type);
    template <typename TT>
    inline TT float2int16(float in_data, uint8_t type);

    inline int16_t pos_float2int(float in_data, uint8_t type);
    inline int16_t vel_float2int(float in_data, uint8_t type);
    inline int16_t tqe_float2int(float in_data, motor_type motor_type);
    inline int16_t rkp_float2int(float in_data, motor_type motor_type);
    inline int16_t rkd_float2int(float in_data, motor_type motor_type);
    inline float pos_int2float(int16_t in_data, uint8_t type);
    inline float vel_int2float(int16_t in_data, uint8_t type);
    inline float tqe_int2float(int16_t in_data, motor_type type);
    inline float pid_scale(float in_data, motor_type motor_type);
    inline int16_t kp_float2int(float in_data, uint8_t type, motor_type motor_type);
    inline int16_t ki_float2int(float in_data, uint8_t type, motor_type motor_type);
    inline int16_t kd_float2int(float in_data, uint8_t type, motor_type motor_type);
    inline int16_t int16_limit(int32_t data);


    void fresh_cmd_int16(float position, float velocity, float torque, float kp, float ki, float kd, float acc, float voltage, float current);

    void position(float position);
    void velocity(float velocity);
    void torque(float torque);
    void voltage(float voltage);
    void current(float current);
    void pos_vel_MAXtqe(float position, float velocity, float torque_max);
    void pos_vel_tqe_kp_kd(float position, float velocity, float torque, float Kp, float Kd);
    void pos_vel_kp_kd(float position, float velocity, float Kp, float Kd);
    void pos_val_acc(float position, float velocity, float acc);
    void pos_vel_rkp_rkd(float position, float velocity, float rKp, float rKd);
    void pos_vel_kp_ki_kd(float position, float velocity, float torque, float kp, float ki, float kd);
    void pos_vel_tqe_rkp_rkd(float position, float velocity, float torque, float rKp, float rKd);

    void fresh_data(int32_t position, int32_t velocity, int32_t torque);
    void fresh_data(int16_t position, int16_t velocity, int16_t torque);
    int get_motor_id() { return id; }
    int get_motor_type() { return type; }
    motor_type get_motor_enum_type() { return type_; }
    int get_motor_num() { return num; }
    /***
     * @brief setting motor type
     * @param type correspond to  different motor type 0~null 1~5046 2~5047_36减速比 3~5047_9减速比
     */
    void set_motor_type(size_t type)
    {
        type_ = static_cast<motor_type>(type);
        // std::cout << "type_:" << type_ << std::endl;
    }
    void set_motor_type(motor_type type)
    {
        type_ = type;
    }
    int get_motor_belong_canport() { return CANport_num; }
    int get_motor_belong_canboard() { return CANboard_num; }
    cdc_acm_rx_message_t *return_cmd_p() { return &cmd; }
    motor_pos_val_tqe_rpd_s *return_pos_val_tqe_rpd_p() { return &cmd_int16_5param; }
    size_t return_size_motor_pos_val_tqe_rpd_s() { return sizeof(motor_pos_val_tqe_rpd_s); }
    motor_back_t *get_current_motor_state() { return &data; }
    std::string get_motor_name(){return motor_name;}
};
#endif