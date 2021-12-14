/**
 * Copyright (c) 2021 Prannoy Namala, Rishabh Mukund, Dani Lerner
 *
 * @file task.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @author Prannoy Namala (pnamala@umd.edu)
 * @author Rishabh Mukund (rmukund@umd.edu)
 * @brief RosSwarmMaster class declaration
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
#include <std_msgs/UInt16.h>
#include <std_srvs/Empty.h>
#include <string>
#include <array>
#include <vector>
#include <unordered_map>

#include "warehouse_swarm/Crate.h"
#include "warehouse_swarm/SwarmConnect.h"
#include "./swarm_master.hpp"

class RosSwarmMaster {
 private:
  SwarmMaster master;
  ros::NodeHandle nh;
  ros::ServiceServer swarm_reset_server;
  ros::ServiceServer swarm_connect_server;
  ros::Subscriber swarm_task_subscriber;
  std::unordered_map<int, ros::Publisher> all_task_publisher;
  std::unordered_map<int, ros::Publisher> all_site_ready_pub;
  std::unordered_map<int, ros::Subscriber> robot_site_waiting_pub;

  /**
    * @brief Get task callback function for swarm task service server
    *
    * @param crate - warehouse_swarm::Crate::ConstPtr&
    */
  void get_task_callback(const warehouse_swarm::Crate::ConstPtr& crate);

  /**
    * @brief Swarm connect callback function for swarm connect service server
    *
    * @param req - first param
    * @param resp - second param
    *
    * @return bool
    */
  bool swarm_connect_callback(warehouse_swarm::SwarmConnect::Request& req,
                              warehouse_swarm::SwarmConnect::Response& resp);

  /**
    * @brief Swarm connect callback function for swarm connect service server
    *
    * @param std_srvs::Empty::Request& - first param
    * @param std_srvs::Empty::Response& - second param
    *
    * @return bool
    */
  bool swarm_reset_callback(std_srvs::Empty::Request&,
                            std_srvs::Empty::Response&);

  /**
    * @brief Robot waiting topic callback
    *
    * @param robot_id
    */
  void get_robot_waiting_callback(const std_msgs::UInt16::ConstPtr& robot_id);

 public:
   /**
     * @brief Constructor for RosSwarmMaster
     *
     * @param designator - first param
     * @param _weight_per_robot - second param
     *
     */
  RosSwarmMaster(AssignmentDesignator* designator,
    double _weight_per_robot=2.0) :
          master{designator, _weight_per_robot} {
      ROS_INFO_STREAM("Spinning up RosSwarmMaster.");
      swarm_reset_server = nh.advertiseService(
        "swarm_reset", &RosSwarmMaster::swarm_reset_callback, this);
      swarm_connect_server = nh.advertiseService(
        "swarm_connect", &RosSwarmMaster::swarm_connect_callback, this);
      swarm_task_subscriber = nh.subscribe("payload_details", 100,
          &RosSwarmMaster::get_task_callback, this);
  }
  ~RosSwarmMaster() {
      ROS_INFO_STREAM("Shutting down RosSwarmMaster.");
  }

  /**
    * @brief Call SwarmMaster assign_robots_to_crate
    *
    * @param pos_init
    * @return int
    */
  int add_robot_to_swarm(std::array<double, 2> pos_init);

  /**
    * @brief Call SwarmMaster assign_robots_to_crate
    *
    * @param crate
    * @return int
    */
  int add_crate_to_system(const Crate& crate);

  /**
    * @brief Call SwarmMaster assign_robots_to_crate
    *
    */
  std::shared_ptr<std::vector<Task> > assign_robots();

  /**
   * @brief Startup the swarm
   *
   * @param duration
   */
  std::shared_ptr<std::vector<Task> > startup(double duration=10,
    double hz=20, double timeout=-1);

  /**
   * @brief Get the swarm master object. FOR TESTING ONLY
   *
   * @return SwarmMaster*
   */
  SwarmMaster* get_swarm_master();

};
