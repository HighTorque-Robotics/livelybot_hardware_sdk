#include "ros/ros.h"
#include "serial_struct.h"
#include "hardware/robot.h"
#include <iostream>
#include <fstream>
#include <thread>
#include <condition_variable>

std::array<double, 2> ankle_ik(double d=0.036, double L1=0.025, double h1=0.112, double h2=0.065, double tx=0, double ty=0)
{
    double cx = cos(tx);
    double sx = sin(tx);
    double cy = cos(ty);
    double sy = sin(ty);

    double AL = -L1 * L1 * cy + L1 * d * sx * sy;
    double BL = -L1 * L1 * sy + L1 * h1 - L1 * d * sx * cy;
    double CL = -(L1 * L1 + d * d - d * d * cx - L1 * h1 * sy - d * h1 * sx * cy);

    double LenL = sqrt(AL * AL + BL * BL);

    double AR = -L1 * L1 * cy - L1 * d * sx * sy;
    double BR = -L1 * L1 * sy + L1 * h2 + L1 * d * sx * cy;
    double CR = -(L1 * L1 + d * d - d * d * cx - L1 * h2 * sy + d * h2 * sx * cy);

    double LenR = sqrt(AR * AR + BR * BR);

    if (LenL <= abs(CL) || LenR <= abs(CR))
    {
        return {0.0, 0.0};
    }
    else
    {
        double tL_1 = asin(CL / LenL) - asin(AL / LenL);
        double tL_2 = asin(CL / LenL) + acos(BL / LenL);

        double tR_1 = asin(CR / LenR) - asin(AR / LenR);
        double tR_2 = asin(CR / LenR) + acos(BR / LenR);

        assert(fabs(tL_1 - tL_2) < 1e-3 && "tL_1 - tL_2 > 1e-3");
        assert(fabs(tR_1 - tR_2) < 1e-3 && "tR_1 - tR_2 > 1e-3");

        return {tL_1, tR_1};
    }
}

void writedata2file(std::vector<float>& poss,std::string path)
{
    std::ofstream file(path, std::ios::app);
    if (!file.is_open()) {
        std::cerr << "无法打开文件用于写入\n";
        return;
          
    }
    for(auto pos:poss)
    file << pos << " ";
    file<<"\n";
    file.close();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_motor_run");
    ros::NodeHandle n;
    ros::Rate r(50);
    livelybot_serial::robot rb;
    std::vector<motor_type> type_list{motor_type::_4538, motor_type::_4538, motor_type::_5047_36, motor_type::_5047_36, motor_type::_5047_36, motor_type::_5047_36,
                                      motor_type::_4538, motor_type::_4538, motor_type::_5047_36, motor_type::_5047_36, motor_type::_5047_36, motor_type::_5047_36}; // 124653
    ROS_INFO("\033[1;32mSTART\033[0m");
    // ========================== singlethread send =====================
    size_t n_motors = 12;

    std::ifstream infile("//home//sunteng//HT-MODEL-H-master//walking_data.txt");
    if (!infile)
    {
        std::cerr << "Unable to open file data.txt" << std::endl;
        return false;
    }
    

    float value;
    std::vector<float> data;
    while (infile >> value)
    {
        data.push_back(value);
    }
    infile.close();
    ROS_INFO("data length:%ld",data.size());
    size_t count = 0;

    for (int i = 0; i < type_list.size(); i++)
    {
        rb.Motors[i]->set_motor_type(type_list[i]);
    }
    std::cout << data.size() << std::endl;
    float cmd[12];

    // std::vector<size_t> map_{0, 1, 5, 2, 4, 3, 6, 7, 11, 8, 10, 9}; // 124653
    // 
    std::vector<size_t> map_18dof{0, 1, 2, 3, 4, 5,  9, 10, 11, 12, 13, 14};//123456
    std::vector<size_t> map_20dof{0, 1, 2, 3, 4, 5, 10, 11, 12, 13, 14, 15};//123456
    std::vector<size_t> map_=map_20dof;
    // std::vector<size_t> idx_{1, 2, 4, 6, 5, 3, 1, 2, 4, 6, 5, 3};   // 124653

    std::vector<float> float_vec(n_motors);
    while (ros::ok() && count < data.size() / n_motors)             // 此用法为逐个电机发送控制指令
    {
        // std::cout<<"ddd"<<std::endl;
        int idx = 0;
        for (int i = 0; i < n_motors; i++)
        {
            cmd[i] = data[(count * n_motors) + i];
        }
        ++count;

        // ROS_INFO_STREAM("START");
        /////////////////////////send
        for (size_t i = 0; i < n_motors; i++)
        {
            // if (i==4){
            // rb.Motors[map[i]]->fresh_cmd(0.0, 0.0, 0.0, 10.0, 0.2);
            // }
            if (map_[i] == 0 || map_[i] == 1 || map_[i] == 9 || map_[i] == 10)
            {
                rb.Motors[map_[i]]->fresh_cmd_int16(cmd[i], 0.0, 0.0, 10.0, 0, 0.2, 0, 0, 0);
            }
            else
            {
                rb.Motors[map_[i]]->fresh_cmd_int16(cmd[i], 0.0, 0.0, 10.0, 0, 0.2, 0, 0, 0);
            }
        }
        // rb.Motors[9]->fresh_cmd_int16(0.2, 0.0, 0.0, 1.5, 0.01, 0, 0, 0);
        rb.motor_send_2();
        ////////////////////////recv
        for (size_t i = 0; i < n_motors; i++)
        {
            motor_back_t motor;
            motor = *rb.Motors[i]->get_current_motor_state();
            float_vec[i]=motor.position;
            ROS_INFO_STREAM("ID: " << map_[i] << " Pos: " << motor.position << " torque: " << motor.torque);
        }
        // writedata2file(float_vec,"//home//sunteng//control_ws//src//livelybot_robot//src//livelybot_serial//data.txt");
        r.sleep();
    }

    return 0;
}
