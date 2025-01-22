#ifndef _ROBOT_H_
#define _ROBOT_H_
#include <iostream>
#include "canboard.h"
#include "ros/ros.h"
#include <thread>
#include <initializer_list>
#include <fstream>

#define DYNAMIC_CONFIG_ROBOT
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
        std::string robot_name, Serial_Type, CANboard_type;
        int CANboard_num, Seial_baudrate, SDK_version;
        ros::NodeHandle n;
        std::vector<canboard> CANboards;
        std::vector<std::string> str;
        std::vector<lively_serial *> ser;
        float SDK_version2 = 3.8; // SDK版本
        std::atomic<bool> publish_joint_state;
        ros::Publisher joint_state_pub_;
        std::thread pub_thread_;

        int control_type;

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
        std::vector<canport *> CANPorts;
        std::vector<std::thread> ser_recv_threads;
        int motor_position_limit_flag = 0;
        int motor_torque_limit_flag = 0;

        robot();
        ~robot();

        void publishJointStates();
        void detect_motor_limit();
        void motor_send_2();
        int serial_pid_vid(const char *name, int *pid, int *vid);
        int serial_pid_vid(const char *name);
        std::vector<std::string> list_serial_ports(const std::string& full_prefix);
        void init_ser();
        void set_port_motor_num();
        void send_get_motor_state_cmd();
        void chevk_motor_connection();
        void set_stop();
        void set_reset();
        void set_reset_zero();
        void set_reset_zero(std::initializer_list<int> motors);
        void set_motor_runzero();
        void set_timeout(int16_t t_ms);
        void set_timeout(uint8_t portx, int16_t t_ms);
        

#ifdef DYNAMIC_CONFIG_ROBOT
        void configCallback(robot_dynamic_config_20Config &config, uint32_t level);
        void fresh_cmd_dynamic_config(float pos, float vel, float torque, size_t motor_idx);
        void fresh_cmd_dynamic_config(float pos, float vel, float torque, float kp, float kd, size_t motor_idx);
        void get_motor_state_dynamic_config(float &pos, float &vel, float &torque, size_t motor_idx);
#endif
    };
}
#endif
