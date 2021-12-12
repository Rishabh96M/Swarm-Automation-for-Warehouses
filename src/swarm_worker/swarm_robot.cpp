/**
 * @file SwarmRobot.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief SwarmRobot class declaration
 * @version 0.1
 * @date 2021-12-03
 * 
 * @copyright Copyright (c) 2021 TBD
 * 
 */

#include <queue>
#include <array>
#include <cmath>
#include "../../include/structs/crate.hpp"
#include "../../include/structs/task.hpp"
#include "../../include/swarm_worker/swarm_robot.hpp"

// SwarmRobot::SwarmRobot() {

// }

// SwarmRobot::~SwarmRobot() {

// }

void SwarmRobot::add_to_task_queue(Task task) {
	task_queue.push(task);
}

std::array<double, 3> SwarmRobot::get_pos() {
	return curr_pos;
}

void SwarmRobot::set_id(int a) {
	robot_id = a;
}

std::array<double, 2> SwarmRobot::mecanum_drive_to(std::array<double, 2> waypt3D){
	std::array<double, 2> ret_array {};
	std::array<double, 3> curr_pos = SwarmRobot::get_pos();

	ret_array[0] = waypt3D[0] -curr_pos[0];
	ret_array[1] = waypt3D[1] -curr_pos[1];
	// Making output a unit vector
	ret_array[0] = ret_array[0]/(sqrt(pow(ret_array[0],2)),sqrt(pow(ret_array[1],2)));
	ret_array[1] = ret_array[1]/(sqrt(pow(ret_array[0],2)),sqrt(pow(ret_array[1],2)));
	return ret_array;
}

double SwarmRobot::turn_towards(double waypt2D) {
	std::array<double, 3> curr_pos = SwarmRobot::get_pos();
	return waypt2D-curr_pos[3];
}

double SwarmRobot::set_platform_height(double delta_h) {
	// Get current platform height
	// Update the velocity paramter

}

bool SwarmRobot::is_queue_empty() {
	return task_queue.empty();
}

Task SwarmRobot::get_task_from_queue() {
	auto ret = task_queue.front();
	task_queue.pop();
	return ret;
}