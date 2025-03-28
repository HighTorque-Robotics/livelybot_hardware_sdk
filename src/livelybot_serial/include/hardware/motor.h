#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "../serial_struct.h"
#include <stdint.h>
#include "ros/ros.h"
#include "livelybot_msg/MotorState.h"


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
    _5047_36_2,
    _6056_36_2,
    _5043_20,
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
    int type, id, num, CANport_num, CANboard_num;
    ros::NodeHandle n;
    motor_back_t data;
    ros::Publisher _motor_pub;
    livelybot_msg::MotorState p_msg;
    std::string motor_name;
    motor_type type_ = motor_type::null;
    cdc_tr_message_s *p_cdc_tx_message = NULL;
    int id_max = 0;
    int control_type = 0;
    pos_vel_convert_type pos_vel_type = radian_2pi; 
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
    motor(int _motor_num, int _CANport_num, int _CANboard_num, cdc_tr_message_s *_p_cdc_tx_message, int _id_max);
    ~motor() {}

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
    void set_motorout(int16_t t_ms);
    void pos_vel_MAXtqe(float position, float velocity, float torque_max);
    void pos_vel_tqe_kp_kd(float position, float velocity, float torque, float Kp, float Kd);
    void pos_vel_kp_kd(float position, float velocity, float Kp, float Kd);
    void pos_vel_acc(float position, float velocity, float acc);
    void pos_vel_rkp_rkd(float position, float velocity, float rKp, float rKd);
    void pos_vel_kp_ki_kd(float position, float velocity, float torque, float kp, float ki, float kd);
    void pos_vel_tqe_rkp_rkd(float position, float velocity, float torque, float rKp, float rKd);

    void fresh_data(int16_t position, int16_t velocity, int16_t torque);

    int get_motor_id();
    int get_motor_type();
    motor_type get_motor_enum_type();
    int get_motor_num();
    void set_motor_type(size_t type);
    void set_motor_type(motor_type type);
    int get_motor_belong_canport();
    int get_motor_belong_canboard();
    motor_pos_val_tqe_rpd_s *return_pos_val_tqe_rpd_p();
    size_t return_size_motor_pos_val_tqe_rpd_s();
    motor_back_t *get_current_motor_state();
    std::string get_motor_name();
};
#endif