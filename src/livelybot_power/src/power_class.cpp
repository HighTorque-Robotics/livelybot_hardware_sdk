#include <iostream>
#include <ros/ros.h>
#include "livelybot_power.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "power_mission");
    ros::NodeHandle n;
    livelybot_can::Power_Board power;
    power.run(n);
}