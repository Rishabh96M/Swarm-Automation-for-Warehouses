/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2021-12-09
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <vector>
#include "../../include/structs/crate.hpp"
#include "warehouse_swarm/Crate.h"
#include "geometry_msgs/Point.h"
#include "../../include/task_orchestrator/task_orchestrator.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_orchestrator");
    std::vector<Crate> crates;
    crates.emplace_back(Crate({5.0, 0.0, 0.25}, {-5.0, 0.0, 0.25}, {0.6, 0.6}, 4));
    crates.emplace_back(Crate({5.0, 2.0, 0.25}, {-5.0, 2.0, 0.25}, {0.6, 0.6}, 8));
    crates.emplace_back(Crate({5.0, -2.0, 0.25}, {-5.0, -2.0, 0.25}, {0.6, 0.6}, 8));
    crates.emplace_back(Crate({5.0, 4.0, 0.25}, {-5.0, 4.0, 0.25}, {0.6, 0.6}, 6));
    crates.emplace_back(Crate({5.0, -4.0, 0.25}, {-5.0, -4.0, 0.25}, {0.6, 0.6}, 6));
    crates.emplace_back(Crate({5.0, 6.0, 0.25}, {-5.0, 6.0, 0.25}, {0.6, 0.6}, 4));
    crates.emplace_back(Crate({5.0, -6.0, 0.25}, {-5.0, -6.0, 0.25}, {0.6, 0.6}, 4));
    TaskOrchestrator taskOrch;
    taskOrch.publish_full_task_list(crates);
    return 0;
}

bool TaskOrchestrator::publish_full_task_list(std::vector<Crate>& crates) {
  std::vector<int>::iterator i;
  payload_pub = nh.advertise<warehouse_swarm::Crate>("/payload_details", 1000);
  for (auto i : crates){
    warehouse_swarm::Crate crate_msg;
    crate_msg.mass = i.mass;
    crate_msg.x_len = i.base_footprint[0];
    crate_msg.y_len = i.base_footprint[1];

    geometry_msgs::Point start_pos;
    start_pos.x = i.start_pos[0];
    start_pos.y = i.start_pos[1];
    start_pos.z = i.start_pos[2];

    geometry_msgs::Point end_pos;
    end_pos.x = i.goal_pos[0];
    end_pos.y = i.goal_pos[1];
    end_pos.z = i.goal_pos[2];

    crate_msg.start_pos = start_pos;
    crate_msg.goal_pos = end_pos;

    payload_pub.publish(crate_msg);
  }
  return true;
}
