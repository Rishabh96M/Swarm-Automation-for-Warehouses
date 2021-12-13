/**
 * @file TaskOrchestrator.hpp
 * @author Dani Lerner (you@domain.com)
 * @brief TaskOrchestrator class declaration
 * @version 0.1
 * @date 2021-12-03
 *
 * @copyright Copyright (c) 2021 TBD
 *
 */

#pragma once

#include <vector>
#include <ros/ros.h>
#include <string>

#include "../structs/crate.hpp"

class TaskOrchestrator {
 public:
    TaskOrchestrator(/* args */) {}
    ~TaskOrchestrator() {}

    ros::Publisher payload_pub;
    ros::NodeHandle nh;

    /**
     * @brief Publishes full task list to task service
     *
     * @return true
     * @return false
     */
    bool publish_full_task_list(const std::vector<Crate>& crates);
};
