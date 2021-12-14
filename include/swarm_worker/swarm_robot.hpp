/**
 * Copyright (c) 2021 Prannoy Namala, Rishabh Mukund, Dani Lerner
 *
 * @file task.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @author Prannoy Namala (pnamala@umd.edu)
 * @author Rishabh Mukund (rmukund@umd.edu)
 * @brief SwarmRobot class declaration
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


#pragma once

#include <queue>
#include <array>
#include "../structs/crate.hpp"
#include "../structs/task.hpp"

class SwarmRobot {
 protected:
    int robot_id;
    Crate designated_crate;
    std::queue<Task> task_queue;
    std::array<double, 3> curr_pos;
    double platform_height{};
 public:
    SwarmRobot(/* args */) {}
    ~SwarmRobot() {}

    /**
     * @brief Set the id int
     *
     */
    void set_id(int);

    /**
     * @brief Add task to task queue
     *
     * @param task
     */
    void add_to_task_queue(Task task);

    /**
     * @brief Get robot position in world
     *
     * @return std::array<double, 3> - x, y, platform height
     */
    std::array<double, 3> get_pos();

    /**
     * @brief Drive to given waypoint with mecanum wheels
     *
     * @param waypt3D
     * @return std::array<double, 2>
     */
    std::array<double, 2> mecanum_drive_to(std::array<double, 2> waypt3D);

    /**
     * @brief Turn towards given waypoint
     *
     * @param waypt2D
     * @return double
     */
    double turn_towards(double waypt2D);

    /**
     * @brief Set the platform height
     *
     * @param delta_h
     * @return double
     */
    double set_platform_height(double delta_h);

    /**
     * @brief Get first task from task queue
     *
     * @param task
     */
    Task get_task_from_queue();

    /**
     * @brief to check if queue is empty
     *
     * @return bool
     */
    bool is_queue_empty();

    /**
     * @brief get robot id
     *
     * @return int
     */
    int get_robot_id();
};
