/**
 * Copyright (c) 2021 Prannoy Namala, Dani Lerner, Rishabh Mukund
 *
 * @file task.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @author Prannoy Namala (pnamala@umd.edu)
 * @author Rishabh Mukund (rmukund@umd.edu)
 * @brief RosSwarmMaster class executable
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

#include <std_msgs/UInt16.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include "warehouse_swarm/Crate.h"
#include "warehouse_swarm/RobotTask.h"
#include "../../include/swarm_master/swarm_master.hpp"
#include "../../include/swarm_master/ros_swarm_master.hpp"
#include <stdexcept>
#include <string>

int RosSwarmMaster::add_robot_to_swarm(std::array<double, 2> pos_init) {
    int id = master.add_robot_to_swarm(pos_init);
    all_task_publisher[id] = nh.advertise<warehouse_swarm::RobotTask>(
        "robot_" + std::to_string(id) + "/task", 10);
    return id;
}

bool RosSwarmMaster::swarm_connect_callback(
        warehouse_swarm::SwarmConnect::Request& req,
        warehouse_swarm::SwarmConnect::Response& resp) {
    resp.id = add_robot_to_swarm({req.x, req.y});
    return true;
}

bool RosSwarmMaster::swarm_reset_callback(std_srvs::Empty::Request&,
                                          std_srvs::Empty::Response&) {
    ROS_INFO_STREAM("RESETTING SWARM");
    master.reset_swarm();
    return true;
}

void RosSwarmMaster::get_robot_waiting_callback
(const std_msgs::UInt16::ConstPtr& robot_id) {
    auto all_waiting_AND_site =
    master.all_robots_at_site_waiting(robot_id->data);
    if (all_waiting_AND_site.first)
        all_site_ready_pub.at(all_waiting_AND_site.second)
        .publish(std_msgs::Empty());
}

int RosSwarmMaster::add_crate_to_system(const Crate& crate) {
    int site_id = master.add_crate_to_system(crate);
    const std::string prefix = "site_" + std::to_string(site_id);
    all_site_ready_pub[site_id] =
    nh.advertise<std_msgs::Empty>(prefix + "/ready", 1);
    robot_site_waiting_pub[site_id] = nh.subscribe(prefix + "/waiting", 10,
        &RosSwarmMaster::get_robot_waiting_callback, this);
    return site_id;
}

void RosSwarmMaster::get_task_callback
(const warehouse_swarm::Crate::ConstPtr& crate_msg) {
    Crate crate({crate_msg->start_pos.x, crate_msg->start_pos.y,
      crate_msg->start_pos.z},
              {crate_msg->goal_pos.x, crate_msg->goal_pos.y,
                crate_msg->goal_pos.z},
              {crate_msg->x_len, crate_msg->y_len}, crate_msg->mass);
    add_crate_to_system(crate);
}

std::shared_ptr<std::vector<Task> > RosSwarmMaster::assign_robots() {
    auto assignments = master.assign_robots_to_crates();
    std::shared_ptr<std::vector<Task> > tasks =
    std::make_shared<std::vector<Task> >();
    for (const auto& assignment : *assignments) {
        auto robot_tasks = master.break_down_assignment(assignment);
        for (const auto& task : *robot_tasks) {
            warehouse_swarm::RobotTask task_msg;
            if (task.task == task.Drive) {
                task_msg.taskType = task_msg.DRIVE;
                task_msg.first_param = task.num_param_dict.at("ToX");
                task_msg.second_param = task.num_param_dict.at("ToY");
                task_msg.third_param = task.num_param_dict.at("ToTheta");
            } else if (task.task == task.MvPlatform) {
                task_msg.taskType = task_msg.MVPLATFORM;
              task_msg.first_param = task.num_param_dict.at("PlatformHeight");
                task_msg.second_param = -1;
                task_msg.third_param = -1;
            } else if (task.task == task.Wait) {
                task_msg.taskType = task_msg.WAIT;
                task_msg.first_param = task.num_param_dict.at("AssignmentID");
                task_msg.second_param = -1;
                task_msg.third_param = -1;
            } else {
                throw std::invalid_argument("Unhandled task type!");
            }
            ROS_DEBUG_STREAM("Assigning task to robot " <<assignment.robot_id);
            tasks->push_back(task);
            all_task_publisher.at(assignment.robot_id).publish(task_msg);
        }
    }
    return tasks;
}

std::shared_ptr<std::vector<Task> > RosSwarmMaster::startup(double duration,
  double hz, double timeout) {
    ros::Duration dur(duration);
    ros::Rate rate(hz);
    ros::Time start_time = ros::Time::now();
    ros::Duration dur_timeout(timeout);

    std::shared_ptr<std::vector<Task> > ret{};
    bool assigned = false;
    bool timeout_bool = true;
    ROS_INFO_STREAM("Entering loop");
    while (!assigned && timeout_bool) {
        auto startup_begin_time = ros::Time::now();
        while (ros::ok() && timeout_bool && ros::Time::now() -
        startup_begin_time < dur) {
            timeout_bool = (timeout == -1 || ros::Time::now() -
            start_time < dur_timeout);
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO_STREAM("Checking assign robots");
        ret = assign_robots();
        assigned = !ret->empty();
    }
    ROS_INFO_STREAM("Leaving loop");
    return ret;
}

SwarmMaster* RosSwarmMaster::get_swarm_master() {
    return &master;
}
