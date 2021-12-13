/**
 * @file task_orchestrator.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-12-12
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <vector>
#include "../../include/structs/crate.hpp"
#include "warehouse_swarm/Crate.h"
#include "../../include/task_orchestrator/task_orchestrator.hpp"

bool TaskOrchestrator::publish_full_task_list(const std::vector<Crate>& crates) {
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
