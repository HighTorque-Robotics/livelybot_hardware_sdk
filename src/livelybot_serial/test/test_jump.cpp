#include "ros/ros.h"
#include "serial_struct.h"
#include "hardware/robot.h"
#include <iostream>
#include <thread>
#include <condition_variable>
#include <boost/tokenizer.hpp>
#include <fstream>
#include <string>
#include <vector>

std::vector<std::vector<double>> read_csv_file(std::string filename)
{
    std::ifstream file(filename);
    std::vector<std::vector<double>> data; 
    std::string line;
    while (std::getline(file, line)) {
        boost::tokenizer<boost::escaped_list_separator<char>> tokens(line);
        std::vector<std::string> row(tokens.begin(), tokens.end());
        std::vector<double> row_double;
        for(int i = 0; i < row.size(); i++)
        {
            row_double.push_back(std::stod(row[i]));
        }
        data.push_back(row_double);
        std::cout << std::stod(row[0]) << " " << row[1] << " " << row[2] << std::endl;
    }
    return data;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_motor");
    ros::NodeHandle n;
    ros::Rate r(25);
    livelybot_serial::robot rb;
    std::vector<std::vector<double>> data = read_csv_file("/home/cat/Documents/CPP/cmake_import_csv/src/ik_out.csv");
    //rb.set_motor_runzero();     // 电机上电自动回零
    ROS_INFO("\033[1;32mSTART\033[0m");
    // ========================== singlethread send =====================
    float derta = 0.01;//角度
    int count = 0;
    float angle = 0.0;
    int dir = 1;
    int i = 0;
    ros::Duration(5).sleep();
    while (ros::ok()) // 此用法为逐个电机发送控制指令
    {
        /////////////////////////send
        rb.detect_motor_limit();
        i = 0;
        std::vector<double> pos = {data[count][0], 0, 0, data[count][1], data[count][2], data[count][2]};
        for (motor *m : rb.Motors)
        {   
            i++;
            ROS_INFO("motor id:%d, pos %.2f", i, m->get_current_motor_state()->position);
            m->pos_vel_MAXtqe(pos[i], 0, 100);
        }
        // ROS_INFO(" ");
        rb.motor_send_2();
        angle += 0.001 * dir;
        count ++;
        if (count >= data.size())
        {
            // dir *= -1;
            count = 0;
        }        
        r.sleep();
        ros::spinOnce();
    }

    ROS_INFO_STREAM("END"); 
    // ros::spin();
    return 0;
}
