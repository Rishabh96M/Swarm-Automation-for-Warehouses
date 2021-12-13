/**
 * @file test_swarm_connect.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-12
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "../include/swarm_master/ros_swarm_master.hpp"
#include "../include/swarm_worker/ros_swarm_robot.hpp"

TEST(MasterWorkerInteractionTests, TestSwarmConnect) {
    RosSwarmRobot robot;
    RosSwarmRobot robot1;
    RosSwarmRobot robot2;
    ros::Duration(1).sleep();
   
    robot.connect_to_master();
    robot1.connect_to_master();
    robot2.connect_to_master();

    EXPECT_EQ(robot.get_robot_id(), 0);
    EXPECT_EQ(robot1.get_robot_id(), 1);
    EXPECT_EQ(robot2.get_robot_id(), 2);
}