#include "robot_pai_hw/hardware/lively_serial.h"
lively_serial::~lively_serial()
{
}
#define read_by_Byte 1
void lively_serial::recv()
{
    ROS_INFO_STREAM("start thread");
    while (ros::ok() && init_flag)
    {
        _result = _ser.read(2);
        // ROS_INFO("==================================START");
        // ROS_INFO(_ser.getPort().c_str());
        if (*(uint8_t *)&_result[0] == 0xFD && *(uint8_t *)&_result[1] == 0xFE)
        {
            _result = _ser.read(sizeof(cdc_acm_tx_message_t) - 2);
            // for (size_t i = 0; i < _result.size(); i++)
            // {
            //     printf("0x%02X ",*(uint8_t *)&_result[i]);
            // }
            // std::cout<<std::endl;
            memcpy(&cdc_acm_tx_message.motor_back_raw, (const void *)&_result[0], sizeof(cdc_acm_tx_message_t) - 2);
            // std::cout<< (int)cdc_acm_tx_message.motor_back.ID<<std::endl;

            if (cdc_acm_tx_message.crc16 == crc_ccitt(0x0000, (const uint8_t *)&cdc_acm_tx_message, sizeof(cdc_acm_tx_message_t) - 2))
            {
                auto it = Map_Motors_p.find(cdc_acm_tx_message.motor_back_raw.ID);
                if (it != Map_Motors_p.end())
                {

                    // it->second->fresh_data(cdc_acm_tx_message.motor_back_raw.position,
                    //                        cdc_acm_tx_message.motor_back_raw.velocity,
                    //                        cdc_acm_tx_message.motor_back_raw.torque);

                    // ROS_INFO("%d", it->second->get_motor_id());
                    // ROS_INFO("END");
                }
                else
                {
                    ROS_ERROR("OUT RANGE");
                }
            }
            else
            {
                // ROS_INFO("%X %X",cdc_acm_tx_message.crc16,crc_ccitt(0x0000, (const uint8_t *)&cdc_acm_tx_message, sizeof(cdc_acm_tx_message_t) - 2));
                memset(&cdc_acm_tx_message.motor_back_raw, 0, sizeof(cdc_acm_tx_message_t) - 2);
                ROS_ERROR("CRC ERROR");
            }
        }
        else
        {
            // ROS_ERROR("FRAME HEAD ERROR");
        }
    }
}
uint16_t lively_serial::return_data_len(uint8_t cmd_id)
{
    switch (cmd_id)
    {
    case 6:
        return 42;
        break;

    default:
        return 0;
        break;
    }
}
uint8_t *return_data_pointer(uint8_t cmd_id)
{
    return nullptr;
}
void lively_serial::recv_1for6_42()
{
    uint16_t data_len = 0;
    uint8_t CRC8 = 0;
    uint16_t CRC16 = 0;
    SOF_t SOF = {0};
    cdc_tr_message_data_s cdc_rx_message_data = {0};
    while (ros::ok() && init_flag)
    {
        _ser.read(&(SOF.head), 1); 
        if (SOF.head == 0xF7)      //  head
        {
            _ser.read(&(SOF.cmd), 4);
            if (SOF.CRC8 == Get_CRC8_Check_Sum((uint8_t *)&(SOF.cmd), 3, 0xFF)) // cmd_id
            {
                _ser.read((uint8_t *)&CRC16, 2);
                _ser.read((uint8_t *)&cdc_rx_message_data, SOF.data_len);
                if (CRC16 != crc_ccitt(0xFFFF, (const uint8_t *)&cdc_rx_message_data, SOF.data_len))
                {
                    memset(&cdc_rx_message_data, 0, sizeof(cdc_rx_message_data) / sizeof(int));
                    // ROS_ERROR("CRC ERROR");
                }
                else
                {
                    // printf("cmd %02X  ", SOF.cmd);
                    // for (int i = 0; i < SOF.data_len; i++)
                    // {
                    //     printf("0x%02X ", cdc_rx_message_data.data[i]);
                    // }
                    // printf("\n");
                    // ROS_INFO("AA");

                    switch (SOF.cmd)
                    {
                        case (MODE_RESET_ZERO):
                        case (MODE_CONF_WRITE):
                        case (MODE_CONF_LOAD):
                            *p_mode_flag = SOF.cmd;
                            for (int i = 0; i < SOF.data_len; i++)
                            {
                                // printf("0x%02X ", cdc_rx_message_data.data[i]);
                                p_motor_id->insert(cdc_rx_message_data.data[i]);
                            }
                            break;

                        case(MODE_SET_NUM):
                            *p_port_version = cdc_rx_message_data.data[2];
                            *p_port_version += (float)cdc_rx_message_data.data[3] * 0.1f;
                            break;

                        case(MODE_MOTOR_STATE):
                            for (size_t i = 0; i < SOF.data_len / sizeof(cdc_rx_motor_state_s); i++)
                            {
                                auto it = Map_Motors_p.find(cdc_rx_message_data.motor_state[i].id);
                                if (it != Map_Motors_p.end())
                                {
                                    it->second->fresh_data(cdc_rx_message_data.motor_state[i].pos,
                                                        cdc_rx_message_data.motor_state[i].val,
                                                        cdc_rx_message_data.motor_state[i].tqe);
                                    // ROS_INFO("%d ", motor_state_6.motor_state[i].pos);
                                    // ROS_INFO("%d", it->second->get_motor_id());
                                    // ROS_INFO("END");
                                }
                                else
                                {
                                    // ROS_ERROR("id not find");
                                }
                            }
                            break;
                    }
                }
            }
            else
            {
                // std::cout << "SOF:\n"
                //           << SOF.head << "  " << SOF.cmd << "  " << SOF.data_len << "  " << return_data_len(SOF.cmd) << std::endl;
                // printf("0x%02X  ", Get_CRC8_Check_Sum((uint8_t *)&(SOF.cmd), 3, 0xFF));
                // printf("0x%02X\n", SOF.CRC8);

                // ROS_ERROR("clcl");
            }
        }
    }
}
void lively_serial::send(uint8_t ID, int32_t position, int32_t velocity, int32_t torque, int16_t Kp, int16_t Kd)
{
    // ROS_INFO_STREAM("START");
    cdc_acm_rx_message.motor_cmd.position = position;
    cdc_acm_rx_message.motor_cmd.velocity = velocity;
    cdc_acm_rx_message.motor_cmd.torque = torque;
    cdc_acm_rx_message.motor_cmd.Kp = Kp;
    cdc_acm_rx_message.motor_cmd.Kd = Kd;
    cdc_acm_rx_message.motor_cmd.ID = ID;
    cdc_acm_rx_message.crc16 = crc_ccitt(0x0000, (const uint8_t *)&cdc_acm_rx_message, sizeof(cdc_acm_rx_message_t) - 2);
    uint8_t *byte_ptr = (uint8_t *)&cdc_acm_rx_message;
    // ROS_INFO_STREAM("STEP1");
    // for (size_t i = 0; i < sizeof(cdc_acm_rx_message_t); i++)
    // {
    //     printf("0x%02X ", byte_ptr[i]);
    // }
    // std::cout << std::endl;
    // ROS_INFO_STREAM("STEP2");
    _ser.write((const uint8_t *)&cdc_acm_rx_message, sizeof(cdc_acm_rx_message));
    // ROS_INFO_STREAM("END"); // STEP2 -> END 1.7ms  START -> END 1.71
}
void lively_serial::send(cdc_acm_rx_message_t *_cdc_acm_rx_message)
{
    uint8_t *byte_ptr = (uint8_t *)_cdc_acm_rx_message;
    // ROS_INFO("STEP1 %x",_cdc_acm_rx_message);
    // for (size_t i = 0; i < sizeof(cdc_acm_rx_message_t); i++)
    // {
    //     printf("0x%02X ", byte_ptr[i]);
    // }
    // std::cout << std::endl;
    _ser.write((const uint8_t *)_cdc_acm_rx_message, sizeof(cdc_acm_rx_message_t));
}
void lively_serial::send_2(cdc_tr_message_s *cdc_tr_message)
{
    cdc_tr_message->head.s.crc8 = Get_CRC8_Check_Sum(&(cdc_tr_message->head.data[1]), 3, 0xFF);
    cdc_tr_message->head.s.crc16 = crc_ccitt(0xFFFF, &(cdc_tr_message->data.data[0]), cdc_tr_message->head.s.len);

    // uint8_t *byte_ptr = (uint8_t *)&cdc_tr_message->head.s.head;
    // printf("send:\n");
    // for (size_t i = 0; i < cdc_tr_message->head.s.len + 7; i++)
    // {
    //     printf("0x%.2X ", byte_ptr[i]);
    // }
    // printf("\n\n");

    _ser.write((const uint8_t *)&cdc_tr_message->head.s.head, cdc_tr_message->head.s.len + sizeof(cdc_tr_message_head_s));
}
