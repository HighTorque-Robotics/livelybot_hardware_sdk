#define DYNAMIC_CONFIG_ROBOT
#ifdef DYNAMIC_CONFIG_ROBOT
#include "ros/ros.h"
#include "../include/serial_struct.h"
#include "../include/hardware/robot.h"
#include <iostream>
#include <thread>
#include <condition_variable>

void excu()
{
    livelybot_serial::robot rb;
    size_t n_motors = 12;
    std::vector<std::string> motor_name{"null", "5046", "4538", "5047_36", "5047_9"};
    int cont = 0;
    ros::Rate r(10);
    //rb.set_motor_runzero();     // 电机上电自动回零
    while (ros::ok()) // 此用法为逐个电机发送控制指令
    {
        // ROS_INFO_STREAM("START");
        /////////////////////////send
        for (size_t i = 0; i < n_motors; i++)
        {
            rb.fresh_cmd_dynamic_config(0,0,0,i);

        }

        rb.motor_send_2();
        ////////////////////////recv
        for (size_t i = 0; i < n_motors; i++)
        {
            
            float pos,vel,tau;
            rb.get_motor_state_dynamic_config(pos,vel,tau,i);
            ROS_INFO_STREAM("idx: " << i << " Pos: " << pos << " torque: " << tau << " type: " << motor_name[static_cast<int>(rb.Motors[i]->get_motor_enum_type())]);
        }
        cont++;
        ROS_INFO("count:     %d", cont);
        if (cont == 10000)
        {
            break;
        }
        r.sleep();
    }



}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_reconfigure");
    ros::NodeHandle n;
    
    ROS_INFO("\033[1;32mSTART\033[0m");
    std::thread sdd(excu);
    ros::spin();
    return 0;
}
#endif
