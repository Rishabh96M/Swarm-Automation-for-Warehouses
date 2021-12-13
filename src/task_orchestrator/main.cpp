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
#include <ros/ros.h>
#include "../../include/structs/crate.hpp"
#include "warehouse_swarm/Crate.h"
#include "geometry_msgs/Point.h"
#include "../../include/task_orchestrator/task_orchestrator.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_orchestrator");
    ros::Duration(3).sleep();
    std::vector<Crate> crates;
    crates.emplace_back(Crate({5.0, 0.0, 0.25}, {-5.0, 0.0, 0.25}, {0.6, 0.6}, 4));
    crates.emplace_back(Crate({5.0, 2.0, 0.25}, {-5.0, 2.0, 0.25}, {0.6, 0.6}, 8));
    crates.emplace_back(Crate({5.0, -2.0, 0.25}, {-5.0, -2.0, 0.25}, {0.6, 0.6}, 8));
    crates.emplace_back(Crate({5.0, 4.0, 0.25}, {-5.0, 4.0, 0.25}, {0.6, 0.6}, 6));
    crates.emplace_back(Crate({5.0, -4.0, 0.25}, {-5.0, -4.0, 0.25}, {0.6, 0.6}, 6));
    crates.emplace_back(Crate({5.0, 6.0, 0.25}, {-5.0, 6.0, 0.25}, {0.6, 0.6}, 4));
    crates.emplace_back(Crate({5.0, -6.0, 0.25}, {-5.0, -6.0, 0.25}, {0.6, 0.6}, 4));
    TaskOrchestrator taskOrch;
    ROS_INFO_STREAM("Publishing task list");
    taskOrch.publish_full_task_list(crates);
    return 0;
}
