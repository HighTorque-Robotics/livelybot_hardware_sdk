#include "ros/ros.h"
#include "serial_struct.h"
#include "hardware/robot.h"
#include <iostream>
#include <thread>
#include <condition_variable>
#include "std_msgs/Float32MultiArray.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_motor_feedback");
    ros::NodeHandle n;
    ros::Rate r(300);
    livelybot_serial::robot rb;
    //rb.set_motor_runzero();     // 电机上电自动回零
    ROS_INFO("\033[1;32mSTART\033[0m");
    // ========================== singlethread send =====================
    int count = 0;
    float vel = 0.2f;
    std::vector<std::string> motor_name{"null","5046","4538","5047_36","5047_9"}; 
    ROS_INFO("motor num %ld" ,rb.Motors.size());
    while (ros::ok())
    {
        rb.detect_motor_limit();
        for (motor *m : rb.Motors)
        {
            // m->velocity(vel);
            auto motor_state = m->get_current_motor_state();
            ROS_INFO("motor:%d, position:%f, velocity:%f, torque:%f", motor_state->ID, motor_state->position, motor_state->velocity, motor_state->torque);
        }
        rb.motor_send_2();
        r.sleep();
        ros::spinOnce();
        count ++;
        if (count >= 600)
        {
            count = 0;
            vel *= -1;
        }
    }
    
    return 0;
}