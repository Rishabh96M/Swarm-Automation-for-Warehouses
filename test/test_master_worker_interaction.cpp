
#include <gtest/gtest.h>
#include "../include/swarm_master/ros_swarm_master.hpp"
#include "../include/swarm_worker/ros_swarm_robot.hpp"

TEST(MasterWorkerInteractionTests, TestSwarmConnect) {
    SimpleClosestDesignator designator;
    RosSwarmMaster master(&designator);
    RosSwarmRobot robot;
    RosSwarmRobot robot1;
    RosSwarmRobot robot2;

    master.startup();
    robot.connect_to_master();
    robot1.connect_to_master();
    robot2.connect_to_master();

    EXPECT_EQ(master.get_swarm_master()->get_avail_robots().size(), 3);
    EXPECT_EQ(robot.get_robot_id(), 0);
    EXPECT_EQ(robot1.get_robot_id(), 1);
    EXPECT_EQ(robot2.get_robot_id(), 2);
}