
#include <ros/ros.h>
#include <gtest/gtest.h>
#include "../include/swarm_master/assignment_designator.hpp"
#include "../include/swarm_master/ros_swarm_master.hpp"
#include "../include/swarm_worker/ros_swarm_robot.hpp"
#include "../include/task_orchestrator/task_orchestrator.hpp"

TEST(MasterWorkerInteractionTests, TestSendRobotTask) {
    RosSwarmRobot robot0;
    RosSwarmRobot robot1;

    SimpleClosestDesignator designator;
    RosSwarmMaster master(&designator);

    TaskOrchestrator taskOrch;
    std::vector<Crate> crates{{{5.0, -6.0, 0.25}, {-5.0, -6.0, 0.25}, {0.6, 0.6}, 4}};
    taskOrch.publish_full_task_list(crates);

    int id0 = master.get_swarm_master()->add_robot_to_swarm({robot0.get_pos()[0], robot0.get_pos()[1]});
    int id1 = master.get_swarm_master()->add_robot_to_swarm({robot1.get_pos()[0], robot1.get_pos()[1]});
    
    robot0.register_bot(id0);
    robot1.register_bot(id1);

    master.startup(1, 20, 10);

    ros::Duration dur(10);
    ros::Rate rate(10);
    auto start = ros::Time::now();
    while (ros::Time::now() - start < dur) {
        ros::spinOnce();
        rate.sleep();
    }
    EXPECT_EQ(robot0.get_queue_size(), 7);
    EXPECT_EQ(robot1.get_queue_size(), 7);
}