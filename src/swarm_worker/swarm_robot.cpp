/**
 * Copyright (c) 2021 Prannoy Namala, Rishabh Mukund, Dani Lerner
 *
 * @file task.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @author Prannoy Namala (pnamala@umd.edu)
 * @author Rishabh Mukund (rmukund@umd.edu)
 * @brief SwarmRobot class executable
 * @version 3.0.1
 * @date 2021-12-03
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <queue>
#include <array>
#include <cmath>
#include "../../include/structs/crate.hpp"
#include "../../include/structs/task.hpp"
#include "../../include/swarm_worker/swarm_robot.hpp"

/**
 * @brief Add task to task queue
 *
 * @param task
 */
void SwarmRobot::add_to_task_queue(Task task) {
	task_queue.push(task);
}

/**
 * @brief Get robot position in world
 *
 * @return std::array<double, 3> - x, y, platform height
 */
std::array<double, 3> SwarmRobot::get_pos() {
	return curr_pos;
}

/**
 * @brief Set the id int
 *
 */
void SwarmRobot::set_id(int a) {
	robot_id = a;
}

/**
 * @brief Drive to given waypoint with mecanum wheels
 *
 * @param waypt3D
 * @return std::array<double, 2>
 */
std::array<double, 2> SwarmRobot::mecanum_drive_to(std::array<double, 2>
	waypt3D) {
		std::array<double, 2> ret_array {};
		std::array<double, 3> curr_pos = SwarmRobot::get_pos();

		ret_array[0] = waypt3D[0] -curr_pos[0];
		ret_array[1] = waypt3D[1] -curr_pos[1];
    // Making output a unit vector
		ret_array[0] =
		ret_array[0]/(sqrt(pow(ret_array[0],2)),sqrt(pow(ret_array[1],2)));
		ret_array[1] =
		ret_array[1]/(sqrt(pow(ret_array[0],2)),sqrt(pow(ret_array[1],2)));

		return ret_array;
	}

/**
 * @brief Turn towards given waypoint
 *
 * @param waypt2D
 * @return double
 */
double SwarmRobot::turn_towards(double waypt2D) {
	std::array<double, 3> curr_pos = SwarmRobot::get_pos();
	return waypt2D-curr_pos[3];
}

/**
 * @brief Set the platform height
 *
 * @param delta_h
 * @return double
 */
double SwarmRobot::set_platform_height(double delta_h) {
	// Get current platform height
	// Update the velocity paramter
	return 0.0;
}

/**
 * @brief to check if queue is empty
 *
 * @return bool
 */
bool SwarmRobot::is_queue_empty() {
	return task_queue.empty();
}

/**
 * @brief Get first task from task queue
 *
 * @param task
 */
Task SwarmRobot::get_task_from_queue() {
	auto ret = task_queue.front();
	task_queue.pop();
	return ret;
}

/**
 * @brief get robot id
 *
 * @return int
 */
int SwarmRobot::get_robot_id() {
    return robot_id;
}
