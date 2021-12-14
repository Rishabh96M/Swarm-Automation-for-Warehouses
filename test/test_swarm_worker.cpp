#include <array>
#include <algorithm>
#include <gtest/gtest.h>
#include "../include/swarm_worker/swarm_robot.hpp"

SwarmRobot testworker;

TEST(SwarmWorkerTests, TestGetPos) {
	std::array<double, 3> test_arr = {0,0,0};
	EXPECT_EQ(testworker.get_pos(),test_arr);
}

TEST(SwarmWorkerTests, TestMecanumDriveTo) {
	std::array<double, 2> test_target = {2,2};
	std::array<double, 2> test_lin_vel = testworker.mecanum_drive_to(test_target);
	std::array<double, 2> test_arr = {1,1};
	EXPECT_EQ(test_lin_vel, test_arr);
}

TEST(SwarmWorkerTests, TestTurnTowards) {
	double test_ang_vel = testworker.turn_towards(2);
	EXPECT_EQ(test_ang_vel, 2);
}

TEST(SwarmWorkerTests, TestSetPlatformHeights) {
	// EXPECT_EQ{1,1};
}

TEST(SwarmWorkerTests, TestAddTaskToQueue) {
	std::unordered_map<std::string, double> test_dict;
	test_dict["first_param"] = 1.1;
	test_dict["second_param"] = 1.1;
	test_dict["third_param"] = 0.1;
	Task test_task(Task::Drive, test_dict);
	testworker.add_to_task_queue(test_task);
	Task expected_task = testworker.get_task_from_queue();
	EXPECT_EQ(test_dict, expected_task.num_param_dict);
}
