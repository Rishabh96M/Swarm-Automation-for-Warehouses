/**
 * @file main.cpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Main script for RosSwarmMaster
 * @version 0.1
 * @date 2021-12-06
 *
 * @copyright Copyright (c) 2021 TBD
 *
 */

#include <ros/ros.h>
#include "../../include/swarm_worker/ros_swarm_robot.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "swarm_worker");
    RosSwarmRobot robot;
    robot.connect_to_master();
    robot.run();
    return 0;
}
