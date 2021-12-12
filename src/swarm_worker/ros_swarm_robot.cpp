/**
 * @file RosSwarmRobot.hpp
 * @author Dani Lerner (dalerner@umd.edu)
 * @brief RosSwarmRobot class declaration
 * @version 0.1
 * @date 2021-12-03
 * 
 * @copyright Copyright (c) 2021 TBD
 * 
 */


#include <ros/ros.h>
#include <string>
#include <array>
#include <cmath>
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
#include "LinearMath/btMatrix3x3.h"


RosSwarmRobot::RosSwarmRobot(std::string task_service_topic): 
    task_service_topic(task_service_topic) {
    swarm_connect_client = nh.serviceClient<warehouse_swarm::SwarmConnect>("swarm_connect");
}

RosSwarmRobot::~RosSwarmRobot() {}

bool RosSwarmRobot::connect_to_master() {
  std::array<double, 3> pos = RosSwarmRobot::get_pos();
  warehouse_swarm::SwarmConnect srv;

  int id {};

  srv.request.x = pos.at(0);
  srv.request.y = pos.at(1);
  if (swarm_connect_client.call(srv)) {
    id = srv.response.id;
    RosSwarmRobot::set_id(id);
    // ROS_INFO("ID Given: %ld", id);
  } else {
    ROS_FATAL_STREAM("Failed to call service swarm_connect");
    return false;
  }

  pos_sub = nh.subscribe("robot_"+std::to_string(id)+"/odom", 1000, &RosSwarmRobot::update_robot_pos, this);
  vel_pub = nh.advertise<geometry_msgs::Twist>("robot_"+std::to_string(id)+"/cmd_vel", 1000);
  // Should be a client right but initialized as a publisher in h
  task_sub = nh.subscribe("robot_" + std::to_string(id) + "/" + task_service_topic, 1000, &RosSwarmRobot::get_task_callback, this);


  return true;
}

void RosSwarmRobot::update_robot_pos(const nav_msgs::Odometry::ConstPtr& msg) {
    auto pose = msg->pose;
    auto position = pose.pose.position;
    auto orirntation = pose.pose.orientation;

    curr_pos = {position.x, position.y, 0};
}

bool RosSwarmRobot::drive_mecanum(std::array<double, 2> target) {
    ros::Rate loop_rate(1);
    // Modify this!!
    double error = sqrt(pow(curr_pos[0] - target[0],2)),sqrt(pow(curr_pos[1] - target[1],2));
    while (error < 0.1) {
        std::array<double, 2> vel_values = RosSwarmRobot::mecanum_drive_to(target);
        // Publish a position or velocity to robot_id/cmd_vel
        geometry_msgs::Twist velocity;
        velocity.linear.x = vel_values[0];
        velocity.linear.y = vel_values[1];
        vel_pub.publish(velocity);
        loop_rate.sleep();
        ros::spinOnce();
    }
}

bool RosSwarmRobot::turn_mecanum(double z) {
    ros::Rate loop_rate(1);
    // Modify this!!
    double error = curr_pos[3] - z;
    while (error < 0.1) {
        double ang_vel_value = turn_towards(z);
        // Publish a position or velocity to robot_id/cmd_vel
        geometry_msgs::Twist velocity;
        velocity.angular.z = ang_vel_value;
        vel_pub.publish(velocity);
        loop_rate.sleep();
        ros::spinOnce();
    }
}

bool RosSwarmRobot::publish_platform_height(double z) {
    // Check for the limit for the max height
    // Publish height to the joint 
    double a = RosSwarmRobot::set_platform_height(z);
    // Publish a to which topic?  -  
}

void RosSwarmRobot::get_task_callback(const warehouse_swarm::RobotTask::ConstPtr& task_msg){
    typedef std::unordered_map<std::string, double> commandDict;
    
    if (task_msg->taskType == task_msg->DRIVE) {
        Task task(Task::Drive, commandDict{{"ToX", task_msg->first_param}, {"ToY", task_msg->second_param}, {"ToTheta", task_msg->third_param}});
        RosSwarmRobot::add_to_task_queue(task);
    } else if (task_msg->taskType == task_msg->MVPLATFORM) {
        Task task(Task::MvPlatform, commandDict{{"PlatformHeight", task_msg->first_param}});
        RosSwarmRobot::add_to_task_queue(task);
    } else if (task_msg->taskType == task_msg->WAIT) {
        Task task(Task::Wait, commandDict{{"AssignmentID", task_msg->first_param}});
        RosSwarmRobot::add_to_task_queue(task);
    } else {
        throw std::invalid_argument("Invalid task type given by master!");
    }
}

void RosSwarmRobot::wait(int site_id) {
    ros::Subscriber ready_sub = nh.subscribe("site_" + std::to_string(site_id) + "/ready", 1000, &RosSwarmRobot::is_ready, this);
    ros::Publisher waiting_pub = nh.advertise<std_msgs::UInt16>(std::to_string(site_id) + "/waiting", 1000);
    ros::Rate loop_rate(20);
    waiting_pub.publish(robot_id);
    while (!ready) {
        loop_rate.sleep();
        ros::spinOnce();
    }

    ready = false;
}

void RosSwarmRobot::is_ready(std_msgs::Empty::ConstPtr&) {
    ready = true;
}

void RosSwarmRobot::run() {
    ros::Rate loop_rate(1);

    while(ros::ok()) {
        while(RosSwarmRobot::is_queue_empty()) {
            loop_rate.sleep();
            ros::spinOnce();
        }
        Task task = SwarmRobot::get_task_from_queue();

        if (task.task == Task::Drive) {
            RosSwarmRobot::turn_mecanum(task.num_param_dict["ToTheta"]);
            RosSwarmRobot::drive_mecanum({task.num_param_dict["ToX"],task.num_param_dict["ToY"]});
        } else if (task.task == Task::MvPlatform) {
            RosSwarmRobot::publish_platform_height(task.num_param_dict["PlatformHeight"]);
        } else if (task.task == Task::Wait) {
            RosSwarmRobot::wait(task.num_param_dict["AssignmentID"]);
        } else {
            throw std::invalid_argument("Invalid task type given by master!");
        }
    }     
}


