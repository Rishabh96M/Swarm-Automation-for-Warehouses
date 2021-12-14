/**
 * Copyright (c) 2021 Prannoy Namala, Rishabh Mukund, Dani Lerner
 *
 * @file task.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @author Prannoy Namala (pnamala@umd.edu)
 * @author Rishabh Mukund (rmukund@umd.edu)
 * @brief RosSwarmRobot class declaration
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

#include <ros/ros.h>
#include <string>
#include <array>
#include "./swarm_robot.hpp"
#include <warehouse_swarm/SwarmConnect.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <nav_msgs/Odometry.h>
#include <warehouse_swarm/RobotTask.h>


class RosSwarmRobot : public SwarmRobot {
 private:
   ros::NodeHandle nh;
   std::string nexus_id;
    std::string task_service_topic;
    ros::ServiceClient swarm_connect_client;
    ros::ServiceServer task_server;
    ros::Publisher vel_pub;
    ros::Subscriber pos_sub;
    ros::Subscriber task_sub;
    bool ready = false;

 public:
   /**
    * @brief Constructor for RosSwarmRobot
    *
    * @param int
    */
    RosSwarmRobot(std::string _nexus_id,
      std::string task_service_topic="/task") :
         nexus_id{_nexus_id},
         task_service_topic(task_service_topic) {
      swarm_connect_client =
      nh.serviceClient<warehouse_swarm::SwarmConnect>("swarm_connect");
    }
    ~RosSwarmRobot() {}

    /**
     * @brief register_bot
     *
     * @param int
     */
   void register_bot(int id);

    /**
     * @brief Connect to swarm master
     *
     * @return bool
     */
    bool connect_to_master();

    /**
     * @brief Connect to swarm master
     *
     * @param msg
     * @return false
     */
    void update_robot_pos(const nav_msgs::Odometry::ConstPtr& msg);

    /**
     * @brief Drive with mecanum wheels
     *
     * @return bool
     */
    bool drive_mecanum(std::array<double, 2>);

    /**
     * @brief Turn with mecanum wheels
     *
     * @return bool
     */
    bool turn_mecanum(double);

    /**
     * @brief Set the platform height
     *
     * @return bool
     */
    bool publish_platform_height(double);

    /**
     * @brief Set the platform height
     *
     * @return bool
     */
    void get_task_callback(const warehouse_swarm::RobotTask::ConstPtr&
      task_msg);

    /**
     * @brief Set the platform height
     *
     * @return bool
     */
    void wait(int site_id);

    /**
     * @brief Set the platform height
     *
     * @return bool
     */
    void is_ready(const std_msgs::Empty::ConstPtr&);

    /**
     * @brief Set the platform height
     *
     * @return bool
     */
    void run();

    /**
     * @brief Set the platform height
     *
     * @return bool
     */
    int get_queue_size();

};
