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

#include <vector>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <std_srvs/Empty.h>
#include "warehouse_swarm/RobotTask.h"
#include "../include/structs/task.hpp"
#include "../include/swarm_master/ros_swarm_master.hpp"
#include "../include/swarm_worker/ros_swarm_robot.hpp"
#include "../include/task_orchestrator/task_orchestrator.hpp"

void reset_swarm() {
    static ros::NodeHandle nh;
    static ros::ServiceClient reset = nh.serviceClient<std_srvs::Empty>("swarm_reset");
    std_srvs::Empty empty;
    reset.call(empty);
    ros::Duration(3).sleep();
}

TEST(SwarmConnectTest, TestSwarmConnect) {
    RosSwarmRobot robot("nexus_1");
    RosSwarmRobot robot1("nexus_2");
    RosSwarmRobot robot2("nexus_3");
    ros::Duration(1).sleep();
   
    robot.connect_to_master();
    robot1.connect_to_master();
    robot2.connect_to_master();

    EXPECT_EQ(robot.get_robot_id(), 0);
    EXPECT_EQ(robot1.get_robot_id(), 1);
    EXPECT_EQ(robot2.get_robot_id(), 2);
}
