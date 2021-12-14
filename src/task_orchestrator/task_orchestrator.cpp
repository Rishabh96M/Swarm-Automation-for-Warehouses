/**
 * Copyright (c) 2021 Prannoy Namala, Rishabh Mukund, Dani Lerner
 *
 * @file task.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @author Prannoy Namala (pnamala@umd.edu)
 * @author Rishabh Mukund (rmukund@umd.edu)
 * @brief TaskOrchestrator class executable
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
#include "../../include/structs/crate.hpp"
#include "warehouse_swarm/Crate.h"
#include "../../include/task_orchestrator/task_orchestrator.hpp"

/**
 * @brief Publishes full task list to task service
 *
 * @return true
 * @return false
 */
bool TaskOrchestrator::publish_full_task_list
(const std::vector<Crate>& crates) {
  for (const auto& crate : crates){
    warehouse_swarm::Crate crate_msg;
    crate_msg.mass = crate.mass;
    crate_msg.x_len = crate.base_footprint[0];
    crate_msg.y_len = crate.base_footprint[1];

    geometry_msgs::Point start_pos;
    start_pos.x = crate.start_pos[0];
    start_pos.y = crate.start_pos[1];
    start_pos.z = crate.start_pos[2];

    geometry_msgs::Point end_pos;
    end_pos.x = crate.goal_pos[0];
    end_pos.y = crate.goal_pos[1];
    end_pos.z = crate.goal_pos[2];

    crate_msg.start_pos = start_pos;
    crate_msg.goal_pos = end_pos;

    payload_pub.publish(crate_msg);
  }
  return true;
}
