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

#include <ros/ros.h>
#include <string>
#include <array>
#include <cmath>
#include <math.h>
#include "warehouse_swarm/RobotTask.h"
#include "warehouse_swarm/SwarmConnect.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <std_msgs/Empty.h>
#include <std_msgs/UInt16.h>
#include <warehouse_swarm/RobotTask.h>
#include "../../include/swarm_worker/ros_swarm_robot.hpp"
#include "../../include/structs/task.hpp"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Matrix3x3.h>

/**
 * @brief register_bot
 *
 * @param int
 */
void RosSwarmRobot::register_bot(int id) {
  set_id(id);
  pos_sub = nh.subscribe(nexus_id + "/odom", 1000,
  &RosSwarmRobot::update_robot_pos, this);
  vel_pub = nh.advertise<geometry_msgs::Twist>(nexus_id + "/cmd_vel", 1000);
  task_sub = nh.subscribe("robot_" + std::to_string(id) + "/" +
  task_service_topic, 1000, &RosSwarmRobot::get_task_callback, this);
}

/**
 * @brief Connect to swarm master
 *
 * @return bool
 */
bool RosSwarmRobot::connect_to_master() {
  std::array<double, 3> pos = RosSwarmRobot::get_pos();
  warehouse_swarm::SwarmConnect srv;

  int id {};

  srv.request.x = pos[0];
  srv.request.y = pos[1];
  ROS_INFO_STREAM("Trying to connect to swarm: " <<
  swarm_connect_client.exists());
  if (swarm_connect_client.exists()) {
    swarm_connect_client.call(srv);
    id = srv.response.id;
    ROS_INFO_STREAM("Connecting robot_" << id << " to swarm");
    RosSwarmRobot::set_id(id);
    // ROS_INFO("ID Given: %ld", id);
  } else {
    ROS_FATAL_STREAM("Failed to call service swarm_connect");
    return false;
  }

  register_bot(id);
  return true;
}

/**
 * @brief Connect to swarm master
 *
 * @param msg
 * @return false
 */
void RosSwarmRobot::update_robot_pos(const nav_msgs::Odometry::ConstPtr& msg) {
    auto pose = msg->pose;
    auto position = pose.pose.position;
    auto orientation = pose.pose.orientation;

    auto yaw = tf::getYaw(orientation);
    curr_pos = {position.x, position.y, yaw};
}

/**
 * @brief Drive with mecanum wheels
 *
 * @return bool
 */
bool RosSwarmRobot::drive_mecanum(std::array<double, 2> target) {
    ROS_INFO_STREAM(nexus_id << " :Moving forward");
    ros::Rate loop_rate(25);
    // Modify this!!
    double error = target[0] - curr_pos[0];
    while (error > 0.1) {
        std::array<double, 2> vel_values =
        RosSwarmRobot::mecanum_drive_to(target);
        // Publish a position or velocity to robot_id/cmd_vel
        geometry_msgs::Twist velocity;
        velocity.linear.x = 0.2;
        vel_pub.publish(velocity);
        loop_rate.sleep();
        ros::spinOnce();
        error = target[0] - curr_pos[0];
    }
    geometry_msgs::Twist vel;
    vel_pub.publish(vel);
    ROS_INFO_STREAM(nexus_id << " :Reached Position");
    return true;
}

/**
 * @brief Turn with mecanum wheels
 *
 * @return bool
 */
bool RosSwarmRobot::turn_mecanum(double z) {
    ROS_INFO_STREAM(nexus_id << " :starting turn");
    ros::Rate loop_rate(25);
    if (z < 0) {
      z = M_PI - z;
    }
    // Modify this!!
    double error = z - curr_pos[2];
    while (abs(error)  > 0.1) {
        double ang_vel_value = RosSwarmRobot::turn_towards(z);
        // Publish a position or velocity to robot_id/cmd_vel
        geometry_msgs::Twist velocity;
        velocity.angular.z = 0.3;
        vel_pub.publish(velocity);
        loop_rate.sleep();
        ros::spinOnce();
        error = z - curr_pos[2];
    }
    geometry_msgs::Twist vel;
    vel_pub.publish(vel);
    ROS_INFO_STREAM(nexus_id << " :Ending turn");
    return true;
}

/**
 * @brief Set the platform height
 *
 * @return bool
 */
bool RosSwarmRobot::publish_platform_height(double z) {
    // Check for the limit for the max height
    // Publish height to the joint
    double a = RosSwarmRobot::set_platform_height(z);
    // Publish a to which topic?  -
    return true;
}

/**
 * @brief Set the platform height
 *
 * @return bool
 */
void RosSwarmRobot::get_task_callback(const
  warehouse_swarm::RobotTask::ConstPtr& task_msg) {
    typedef std::unordered_map<std::string, double> commandDict;

    if (task_msg->taskType == task_msg->DRIVE) {
        Task task(Task::Drive, commandDict{{"ToX", task_msg->first_param},
        {"ToY", task_msg->second_param}, {"ToTheta", task_msg->third_param}});
        RosSwarmRobot::add_to_task_queue(task);
    } else if (task_msg->taskType == task_msg->MVPLATFORM) {
        Task task(Task::MvPlatform,
          commandDict{{"PlatformHeight", task_msg->first_param}});
        RosSwarmRobot::add_to_task_queue(task);
    } else if (task_msg->taskType == task_msg->WAIT) {
        Task task(Task::Wait, commandDict{{"AssignmentID",
        task_msg->first_param}});
        RosSwarmRobot::add_to_task_queue(task);
    } else {
        throw std::invalid_argument("Invalid task type given by master!");
    }
}

/**
 * @brief Set the platform height
 *
 * @return bool
 */
void RosSwarmRobot::wait(int site_id) {
    ros::Subscriber ready_sub = nh.subscribe("site_" +
    std::to_string(site_id) + "/ready", 1000, &RosSwarmRobot::is_ready, this);
    ros::Publisher waiting_pub =
    nh.advertise<std_msgs::UInt16>(std::to_string(site_id) + "/waiting", 1000);
    ros::Rate loop_rate(20);
    std_msgs::UInt16 msg;
    msg.data = robot_id;
    waiting_pub.publish(msg);
    while (!ready) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    ready = false;
}

/**
 * @brief Set the platform height
 *
 * @return bool
 */
void RosSwarmRobot::is_ready(const std_msgs::Empty::ConstPtr&) {
    ready = true;
}

/**
 * @brief Set the platform height
 *
 * @return bool
 */
void RosSwarmRobot::run() {
    ros::Rate loop_rate(1);

    while (ros::ok()) {
        while (RosSwarmRobot::is_queue_empty()) {
            loop_rate.sleep();
            ros::spinOnce();
        }
        Task task = SwarmRobot::get_task_from_queue();

        if (task.task == Task::Drive) {
            double ang = atan2(task.num_param_dict["ToY"] -
            curr_pos[1], task.num_param_dict["ToX"] - curr_pos[0]);
            RosSwarmRobot::turn_mecanum(ang);
            RosSwarmRobot::drive_mecanum
            ({task.num_param_dict["ToX"], task.num_param_dict["ToY"]});
            RosSwarmRobot::turn_mecanum(task.num_param_dict["ToTheta"]
            * M_PI / 180);
        } else if (task.task == Task::MvPlatform) {
            RosSwarmRobot::publish_platform_height
            (task.num_param_dict["PlatformHeight"]);
        } else if (task.task == Task::Wait) {
            RosSwarmRobot::wait(task.num_param_dict["AssignmentID"]);
        } else {
            throw std::invalid_argument("Invalid task type given by master!");
        }
    }
}

/**
 * @brief Set the platform height
 *
 * @return bool
 */
int RosSwarmRobot::get_queue_size() {
    return task_queue.size();
}
