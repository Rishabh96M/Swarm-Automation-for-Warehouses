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
    reset.call(std_srvs::Empty());
}

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

    reset_swarm();
}

int num_tasks_0 = 0;
void task_cb0(const warehouse_swarm::RobotTask::ConstPtr& task_msg) {
    num_tasks_0++;
}

int num_tasks_1 = 0;
void task_cb1(const warehouse_swarm::RobotTask::ConstPtr& task_msg) {
    num_tasks_1++;
}

TEST(MasterWorkerInteractionTests, TestSendRobotTask) {
    RosSwarmRobot robot0;
    RosSwarmRobot robot1;

    TaskOrchestrator taskOrch;
    std::vector<Crate> crates{{{5.0, -6.0, 0.25}, {-5.0, -6.0, 0.25}, {0.6, 0.6}, 4)}};
    taskOrch.publish_full_task_list(crates);

    robot0.connect_to_master();
    robot1.connect_to_master();
  
    ros::NodeHandle nh;
    auto robot0_task = nh.subscribe("robot_0/task", 100, &task_cb0);
    auto robot0_task = nh.subscribe("robot_0/task", 100, &task_cb1);
    
    ros::Duration dur(2);
    ros::Rate rate(10);
    auto start = ros::Time::now();
    while (ros::Time::now() - start < dur) {
        ros::spinOnce();
        rate.sleep();
    }
    EXPECT_EQ(num_tasks_0, 7);
    EXPECT_EQ(num_tasks_1, 7);

    reset_swarm();
}

