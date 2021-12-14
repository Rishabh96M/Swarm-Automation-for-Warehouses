/**
 * @file main.cpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief Testing main
 * @version 0.1
 * @date 2021-12-05
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <gtest/gtest.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "swarm_connect_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
