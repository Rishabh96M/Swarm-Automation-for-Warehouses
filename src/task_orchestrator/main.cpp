/**
 * Copyright (c) 2021 Prannoy Namala, Rishabh Mukund, Dani Lerner
 *
 * @file task.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @author Prannoy Namala (pnamala@umd.edu)
 * @author Rishabh Mukund (rmukund@umd.edu)
 * @brief TaskOrchestrator main class executable
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

#include <vector>
#include <ros/ros.h>
#include "../../include/structs/crate.hpp"
#include "warehouse_swarm/Crate.h"
#include "geometry_msgs/Point.h"
#include "../../include/task_orchestrator/task_orchestrator.hpp"

/**
 * @brief Main file fot task ochestrator class
 *
 * @return int
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "task_orchestrator");
    std::vector<Crate> crates;
    crates.emplace_back
    (Crate({5.0, 0.0, 0.25}, {-5.0, 0.0, 0.25}, {0.6, 0.6}, 4));
    crates.emplace_back
    (Crate({5.0, 2.0, 0.25}, {-5.0, 2.0, 0.25}, {0.6, 0.6}, 8));
    crates.emplace_back
    (Crate({5.0, -2.0, 0.25}, {-5.0, -2.0, 0.25}, {0.6, 0.6}, 8));
    crates.emplace_back
    (Crate({5.0, 4.0, 0.25}, {-5.0, 4.0, 0.25}, {0.6, 0.6}, 6));
    crates.emplace_back
    (Crate({5.0, -4.0, 0.25}, {-5.0, -4.0, 0.25}, {0.6, 0.6}, 6));
    crates.emplace_back
    (Crate({5.0, 6.0, 0.25}, {-5.0, 6.0, 0.25}, {0.6, 0.6}, 4));
    crates.emplace_back
    (Crate({5.0, -6.0, 0.25}, {-5.0, -6.0, 0.25}, {0.6, 0.6}, 4));
    TaskOrchestrator taskOrch;
    bool waited = ros::Duration(3).sleep();
    ROS_INFO_STREAM("Publishing task list " << waited);
    taskOrch.publish_full_task_list(crates);
    ros::spin();
    return 0;
}
